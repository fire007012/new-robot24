#include "robot_safety/domain.hpp"
#include <algorithm>
#include <sstream>
#include <iomanip>
#include <chrono>
#include <stdexcept>
namespace robot_safety {
namespace {
int64_t steadyNow() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}
std::string json(const std::string & value) {
  std::ostringstream out; out << '"';
  for (unsigned char c : value) {
    if (c == '"' || c == '\\') out << '\\' << c;
    else if (c < 0x20) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << int(c);
    else out << c;
  }
  out << '"'; return out.str();
}
}
Domain::Domain(Clock clock) : clock_(clock ? std::move(clock) : Clock(steadyNow)) {}
void Domain::configure(const std::map<std::string, Axis> & axes,
                       const std::vector<Association> & rules, const std::string & version) {
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto & writer : writers_) if (!writer.second)
    throw std::logic_error("legacy writer already registered in group domain");
  if (version.empty() || axes.empty()) throw std::invalid_argument("empty safety configuration");
  if (state_ == State::Armed || state_ == State::Running)
    throw std::logic_error("halt legacy session before configuring");
  for (const auto & item : groups_)
    if (item.second.state == State::Armed || item.second.state == State::Running)
      throw std::logic_error("halt all groups before configuring");
  const std::set<std::string> names{"arm", "flippers", "base", "gripper"};
  std::set<std::string> present;
  for (const auto & item : axes) {
    if (item.first.empty() || !names.count(item.second.group) || item.second.backend.empty())
      throw std::invalid_argument("invalid axis ownership");
    present.insert(item.second.group);
  }
  if (present != names) throw std::invalid_argument("all four groups must have axes");
  std::set<std::string> seen;
  for (const auto & rule : rules) {
    if (!names.count(rule.source_group) || !names.count(rule.dependent_group) ||
        rule.source_group == rule.dependent_group || rule.category.empty() ||
        rule.condition.empty() || rule.source.empty() ||
        !seen.insert(rule.source_group + "\n" + rule.category + "\n" + rule.dependent_group + "\n" + rule.condition).second)
      throw std::invalid_argument("invalid/duplicate association");
  }
  // Reconfiguration cannot clear a latched event or global emergency.
  if (!events_.empty()) throw std::logic_error("acknowledge safety events before configuring");
  axes_ = axes;
  for (auto & axis : axes_) { axis.second.ready = false; axis.second.reason = "not reported"; }
  associations_ = rules; configuration_version_ = version; conditions_.clear();
  for (const auto & name : names) {
    auto & g = groups_[name]; ++g.epoch; g.state = State::Stopped; g.reason = "configuration changed"; g.blockers.clear();
  }
  refreshGroups();
}
void Domain::refreshGroups() {
  for (auto & item : groups_) {
    item.second.ready = true;
    for (const auto & axis : axes_)
      if (axis.second.group == item.first && !axis.second.ready) item.second.ready = false;
  }
}
void Domain::revoke(const std::string & name, const std::string & reason) const {
  auto & g = groups_.at(name);
  if (g.state != State::Fault) ++g.epoch;
  g.state = State::Fault; g.reason = reason;
}
bool Domain::conditionSatisfied(const Association & rule) const {
  auto it = conditions_.find(rule.condition);
  return it != conditions_.end() && it->second.value == Evidence::Satisfied &&
    it->second.source == rule.source && it->second.valid_until_ns > clock_();
}
std::set<std::string> Domain::scope(const Event & event) const {
  std::set<std::string> result;
  if (event.severity == Severity::GlobalStop) {
    for (const auto & g : groups_) result.insert(g.first);
  } else if (event.severity == Severity::GroupStop) {
    if (!event.axis.empty()) result.insert(axes_.at(event.axis).group);
    if (!event.group.empty()) result.insert(event.group);
    // An axis event with backend metadata is still an axis event, not a bus outage.
    if (result.empty())
      for (const auto & axis : axes_) if (axis.second.backend == event.backend) result.insert(axis.second.group);
  }
  bool changed = true;
  while (changed) {
    changed = false;
    for (const auto & rule : associations_) {
      // Propagated restrictions carry unknown mechanical state. Apply all downstream rules.
      if (result.count(rule.source_group) &&
          (rule.category == "*" || rule.category == event.category || rule.source_group != event.group) &&
          !conditionSatisfied(rule) && result.insert(rule.dependent_group).second) changed = true;
    }
  }
  return result;
}
std::map<std::string, std::pair<std::set<std::string>, std::string>> Domain::restrictions() const {
  std::map<std::string, std::pair<std::set<std::string>, std::string>> out;
  for (const auto & e : events_) out.emplace(e.first, std::make_pair(scope(e.second), e.second.reason));
  // No report is unknown readiness, not evidence of a mechanically safe inactive axis.
  // This only restricts dependent groups; the unprepared source keeps its startup state.
  for (const auto & g : groups_) if (!g.second.ready) {
    Event e; e.group = g.first; e.category = "unavailable";
    auto affected = scope(e); affected.erase(g.first);
    out.emplace("unavailable:" + g.first, std::make_pair(affected,
      g.first + " not ready; associated physical conditions unknown/unsatisfied"));
  }
  return out;
}
void Domain::propagate() const {
  expireOwner();
  for (auto & g : groups_) g.second.blockers.clear();
  for (const auto & entry : restrictions()) {
    for (const auto & name : entry.second.first) {
      auto & g = groups_.at(name);
      g.blockers.push_back(entry.first);
      revoke(name, entry.second.second);
    }
  }
}
void Domain::unprepareAxis(const std::string & name, const std::string & reason) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto & axis = axes_.at(name); axis.ready = false; axis.reason = reason;
  auto & g = groups_.at(axis.group);
  if (g.state == State::Running || g.state == State::Armed) revoke(axis.group, reason);
  refreshGroups(); propagate();
}
void Domain::warning(const std::string & name, bool active, const std::string & reason) {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto & axis = axes_.at(name); const auto id = "warning:" + name;
  if (!active) { events_.erase(id); return; }
  Event e; e.id=id;e.axis=name;e.group=axis.group;e.backend=axis.backend;e.severity=Severity::Warn;
  e.category="feedback_warning";e.reason=reason;e.timestamp_ns=clock_();e.recovery="stable feedback window";
  if (events_.emplace(id,e).second) ++warning_counts_[name];
}
void Domain::reportAxis(const std::string & name, bool ready, const std::string & reason,
                        const std::string & code, const std::string & category) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto & axis = axes_.at(name);
  axis.ready = ready; axis.reason = reason;
  if (!ready) {
    Event event; event.id = "readiness:" + name; event.axis = name; event.group = axis.group;
    event.backend = axis.backend; event.category = category; event.code = code; event.reason = name + ": " + reason;
    event.recovery = "fresh feedback, explicit acknowledgement, arm/start and new command";
    event.timestamp_ns = clock_();
    // Preserve the first occurrence; periodic reports never change healthy epochs.
    events_.emplace(event.id, event);
  }
  refreshGroups(); propagate();
}
void Domain::reportBackend(const std::string & name, bool ready, const std::string & reason) {
  std::lock_guard<std::mutex> lock(mutex_);
  bool found = false;
  for (auto & axis : axes_) if (axis.second.backend == name) {
    found = true;
    // Restoring the bus does not prove that each drive has trustworthy feedback.
    if (!ready) { axis.second.ready = false; axis.second.reason = reason; }
  }
  if (!found) throw std::invalid_argument("unknown backend");
  if (!ready) {
    Event e; e.id = "backend:" + name; e.backend = name; e.category = "transport";
    e.reason = reason; e.timestamp_ns = clock_(); e.recovery = "restore shared resource and affected axes";
    events_.emplace(e.id, e);
  }
  refreshGroups(); propagate();
}
void Domain::condition(const std::string & name, Evidence value, const std::string & source, int64_t until) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (name.empty() || source.empty() || (value != Evidence::Unknown && until <= clock_()))
    throw std::invalid_argument("invalid/expired condition evidence");
  bool known = false;
  for (const auto & rule : associations_) if (rule.condition == name && rule.source == source) known = true;
  if (!known) throw std::invalid_argument("condition source is not configured");
  propagate();  // Commit expiry before accepting renewed evidence; never revive an old session.
  conditions_[name] = {value, source, until}; propagate();
}
void Domain::event(const Event & value) {
  std::lock_guard<std::mutex> lock(mutex_);
  Event e = value;
  switch (e.severity) {
    case Severity::Reject: case Severity::Warn: case Severity::GroupStop: case Severity::GlobalStop: break;
    default: throw std::invalid_argument("invalid event severity");
  }
  if (e.timestamp_ns < 0) throw std::invalid_argument("invalid event timestamp");
  if (e.id.rfind("readiness:", 0) == 0 || e.id.rfind("backend:", 0) == 0 ||
      e.id.rfind("unavailable:", 0) == 0 || e.id.rfind("warning:", 0) == 0)
    throw std::invalid_argument("reserved internal event namespace");
  if (e.id.empty() || e.reason.empty()) throw std::invalid_argument("event id/reason required");
  if (!e.axis.empty()) {
    const auto & a = axes_.at(e.axis);
    if ((!e.group.empty() && e.group != a.group) || (!e.backend.empty() && e.backend != a.backend))
      throw std::invalid_argument("event source ownership mismatch");
    e.group = a.group; e.backend = a.backend;
  }
  if (!e.group.empty() && !groups_.count(e.group)) throw std::invalid_argument("unknown event group");
  if (e.severity == Severity::GroupStop && scope(e).empty()) throw std::invalid_argument("event scope required");
  if (!e.timestamp_ns) e.timestamp_ns = clock_();
  auto old = events_.find(e.id);
  if (old != events_.end()) {
    // An ID identifies one occurrence, not a mutable scope or severity.
    if (old->second.axis != e.axis || old->second.group != e.group || old->second.backend != e.backend ||
        old->second.category != e.category || old->second.code != e.code || old->second.severity != e.severity)
      throw std::invalid_argument("event id reused with different source/classification");
    propagate(); return;
  }
  events_.emplace(e.id, e);
  if (e.severity == Severity::GlobalStop) {
    global_inhibit_ = true; global_reason_ = e.reason;
    if (state_ != State::Fault) ++epoch_;
    state_ = State::Fault; reason_ = e.reason;
  }
  propagate();
}
void Domain::acknowledge(const std::string & id) {
  std::lock_guard<std::mutex> lock(mutex_);
  propagate();
  const auto it = events_.find(id);
  if (it == events_.end()) throw std::invalid_argument("unknown event");
  // Device readiness is necessary, but not sufficient: the caller must explicitly confirm recovery.
  const auto & e = it->second;
  if (e.severity == Severity::GroupStop || e.severity == Severity::GlobalStop) {
    for (const auto & a : axes_)
      if ((!e.axis.empty() && a.first == e.axis) ||
          (e.axis.empty() && !e.group.empty() && a.second.group == e.group) ||
          (e.axis.empty() && e.group.empty() && a.second.backend == e.backend))
        if (!a.second.ready) throw std::logic_error("source not ready");
  }
  events_.erase(it); propagate();  // Fault stays latched until explicit arm.
}
bool Domain::arm(const std::string & name) {
  std::lock_guard<std::mutex> lock(mutex_); propagate();
  auto & g = groups_.at(name);
  if (global_inhibit_ || !ownerFresh() || !g.ready || !g.blockers.empty() || g.state == State::Running) return false;
  if (g.state != State::Armed) ++g.epoch;
  g.state = State::Armed; g.reason = "armed; explicit start and new command required"; return true;
}
bool Domain::run(const std::string & name) {
  std::lock_guard<std::mutex> lock(mutex_); propagate();
  auto & g = groups_.at(name);
  if (global_inhibit_ || !ownerFresh() || !g.ready || !g.blockers.empty() || g.state != State::Armed) return false;
  g.state = State::Running; g.reason = "running"; return true;
}
void Domain::halt(const std::string & name) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto & g = groups_.at(name);
  if (g.state == State::Armed || g.state == State::Running) ++g.epoch;
  if (g.state != State::Fault) { g.state = State::Stopped; g.reason = "halted"; }
}
Domain::Group Domain::snapshot(const std::string & name) const {
  // Permission checks commit evidence expiry under the same lock as the epoch check.
  propagate(); return groups_.at(name);
}
bool Domain::permits(const std::string & name, uint64_t epoch) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto g = snapshot(name);
  return !global_inhibit_ && ownerFresh() && g.ready && g.blockers.empty() && g.state == State::Running && g.epoch == epoch;
}
uint64_t Domain::epoch(const std::string & name) const { std::lock_guard<std::mutex> lock(mutex_); propagate(); return groups_.at(name).epoch; }
Domain::Group Domain::group(const std::string & name) const { std::lock_guard<std::mutex> lock(mutex_); return snapshot(name); }
std::map<std::string, Domain::Group> Domain::groups() const {
  std::lock_guard<std::mutex> lock(mutex_); propagate(); return groups_;
}
void Domain::globalStop(const std::string & reason) {
  std::lock_guard<std::mutex> lock(mutex_);
  global_inhibit_ = true; global_reason_ = reason;
  for (const auto & g : groups_) revoke(g.first, reason);
  if (state_ != State::Fault) ++epoch_;
  state_ = State::Fault; reason_ = reason;
}
bool Domain::resetGlobalStop() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!ownerFresh()) return false;
  for (const auto & e : events_) if (e.second.severity == Severity::GlobalStop) return false;
  owner_expired_ = false;
  global_inhibit_ = false; global_reason_.clear(); return true;
}
bool Domain::ownerFresh() const { return !owner_required_ || owner_deadline_ > clock_(); }
void Domain::expireOwner() const {
  if (owner_required_ && owner_deadline_ > 0 && !owner_expired_ && !ownerFresh()) {
    owner_expired_ = true;
    if (!global_inhibit_) global_reason_ = "runtime owner heartbeat expired";
    global_inhibit_ = true;
    for (const auto & g : groups_) revoke(g.first,"runtime owner heartbeat expired");
  }
}
void Domain::requireOwnerLease() { std::lock_guard<std::mutex> lock(mutex_); owner_required_=true; }
void Domain::renewOwner(int64_t lifetime_ns,int64_t issued_ns) {
  std::lock_guard<std::mutex> lock(mutex_); expireOwner();
  const auto now=clock_();const auto issued=issued_ns?issued_ns:now;
  if (lifetime_ns<=0 || issued<0 || issued>now || issued>INT64_MAX-lifetime_ns || issued+lifetime_ns<=now)
    throw std::invalid_argument("expired/future owner heartbeat");
  owner_deadline_=std::max(owner_deadline_,issued+lifetime_ns);
}
bool Domain::globallyInhibited() const { std::lock_guard<std::mutex> lock(mutex_); expireOwner(); return global_inhibit_; }
std::vector<Domain::Event> Domain::events() const {
  std::lock_guard<std::mutex> lock(mutex_); std::vector<Event> out;
  for (const auto & e : events_) out.push_back(e.second);
  return out;
}
std::string Domain::axisGroup(const std::string & axis, const std::string & backend) const {
  std::lock_guard<std::mutex> lock(mutex_);
  const auto & a = axes_.at(axis);
  if (a.backend != backend) throw std::invalid_argument("axis backend ownership mismatch: " + axis);
  return a.group;
}
void Domain::registerWriter(const std::string & backend, bool grouped) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (backend.empty() || (!grouped && !groups_.empty())) throw std::invalid_argument("mixed writer modes");
  for (const auto & writer : writers_) if (writer.second != grouped) throw std::invalid_argument("mixed writer modes");
  writers_[backend] = grouped;
}
bool Domain::grouped() const { std::lock_guard<std::mutex> lock(mutex_); return !groups_.empty(); }
void Domain::invalidate(const std::string & name) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto & g = groups_.at(name); ++g.epoch;
  if (g.state != State::Fault) { g.state = State::Stopped; g.reason = "operation cancelled/halted"; }
}
void Domain::poll() { std::lock_guard<std::mutex> lock(mutex_); propagate(); }

void Domain::report(const std::string & component, bool ready) {
  std::lock_guard<std::mutex> lock(mutex_);
  ready_[component] = ready;
  if (!ready && !groups_.empty()) {
    const bool moving = std::any_of(groups_.begin(), groups_.end(), [](const auto & item) {
      return item.second.state == State::Armed || item.second.state == State::Running;
    });
    if (moving) {
      global_inhibit_ = true; global_reason_ = component + " lost readiness (legacy report)";
      for (const auto & g : groups_) revoke(g.first, global_reason_);
    }
  }
  if (!ready && (state_ == State::Running || state_ == State::Armed)) {
    state_ = State::Fault; ++epoch_; reason_ = component + " lost readiness";
  }
}
void Domain::fault(const std::string & reason) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!groups_.empty()) {
    global_inhibit_ = true; global_reason_ = reason;
    for (const auto & g : groups_) revoke(g.first, reason);
  }
  if (state_ != State::Fault) ++epoch_;
  state_ = State::Fault; reason_ = reason;
}
bool Domain::arm() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (global_inhibit_ || !groups_.empty()) return false;
  if (!std::all_of(ready_.begin(), ready_.end(), [](const auto & v) { return v.second; })) return false;
  ++epoch_; state_ = State::Armed; reason_ = "armed"; return true;
}
bool Domain::run() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (state_ != State::Armed) return false;
  state_ = State::Running; reason_ = "running"; return true;
}
void Domain::halt() {
  std::lock_guard<std::mutex> lock(mutex_);
  ++epoch_; state_ = State::Stopped; reason_ = "halted";
  for (auto & item : groups_) {
    auto & g = item.second;
    if (g.state == State::Running || g.state == State::Armed) {
      ++g.epoch; g.state = State::Stopped; g.reason = "global halt";
    }
  }
}
bool Domain::permits(uint64_t epoch) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return !global_inhibit_ && state_ == State::Running && epoch == epoch_;
}
uint64_t Domain::epoch() const { std::lock_guard<std::mutex> lock(mutex_); return epoch_; }
Domain::State Domain::state() const { std::lock_guard<std::mutex> lock(mutex_); return state_; }
std::string Domain::reason() const { std::lock_guard<std::mutex> lock(mutex_); return reason_; }
void Domain::observePeriod(int64_t nanoseconds) {
  std::lock_guard<std::mutex> lock(mutex_);
  const bool group_running = std::any_of(groups_.begin(), groups_.end(), [](const auto & item) {
    return item.second.state == State::Running;
  });
  if ((!group_running && state_ != State::Running) || nanoseconds <= 0) return;
  const auto ns = static_cast<uint64_t>(nanoseconds);
  ++samples_; sum_ns_ += ns; max_ns_ = std::max(max_ns_, ns); min_ns_ = std::min(min_ns_, ns);
  if (ns > 7500000) ++late_;  // Report observed misses against 200 Hz; never claim a real-time guarantee.
}
std::string Domain::diagnostics() const {
  std::lock_guard<std::mutex> lock(mutex_); expireOwner();
  std::ostringstream out;
  out << "{\"state\":" << static_cast<int>(state_) << ",\"epoch\":" << epoch_
      << ",\"reason\":" << json(reason_) << ",\"samples\":" << samples_
      << ",\"period_min_ns\":" << (samples_ ? min_ns_ : 0)
      << ",\"period_mean_ns\":" << (samples_ ? sum_ns_ / samples_ : 0)
      << ",\"period_max_ns\":" << max_ns_ << ",\"period_over_7_5ms\":" << late_
      << ",\"configuration_version\":" << json(configuration_version_)
      << ",\"global_inhibit\":" << (global_inhibit_ ? "true" : "false")
      << ",\"owner_required\":" << (owner_required_ ? "true" : "false")
      << ",\"owner_fresh\":" << (ownerFresh() ? "true" : "false")
      << ",\"global_reason\":" << json(global_reason_) << ",\"groups\":{";
  bool first = true;
  for (const auto & entry : groups_) {
    if (!first) out << ','; first = false;
    const auto g = snapshot(entry.first);
    out << json(entry.first) << ":{\"state\":" << static_cast<int>(g.state)
        << ",\"epoch\":" << g.epoch << ",\"ready\":" << (g.ready ? "true" : "false")
        << ",\"reason\":" << json(g.reason) << ",\"blockers\":[";
    for (size_t i = 0; i < g.blockers.size(); ++i) { if (i) out << ','; out << json(g.blockers[i]); }
    out << "]}";
  }
  out << "},\"aggregate_state\":";
  size_t running = 0, armed = 0, faulted = 0;
  for (const auto & entry : groups_) {
    const auto g = snapshot(entry.first);
    running += g.state == State::Running; armed += g.state == State::Armed; faulted += g.state == State::Fault;
  }
  const char * legacy_summary = state_ == State::Running ? "Running" :
    state_ == State::Fault ? "Restricted" : state_ == State::Armed ? "Armed" : "Stopped";
  out << json(global_inhibit_ ? "GloballyInhibited" : groups_.empty() ? legacy_summary :
    running ? (running == groups_.size() ? "Running" : "PartialRunning") :
    faulted ? "Restricted" : armed ? "Armed" : "Stopped");
  out << ",\"axes\":{"; first = true;
  for (const auto & entry : axes_) {
    if (!first) out << ','; first = false;
    const auto & a = entry.second;
    out << json(entry.first) << ":{\"group\":" << json(a.group) << ",\"backend\":" << json(a.backend)
        << ",\"ready\":" << (a.ready ? "true" : "false")
        << ",\"warning_count\":" << (warning_counts_.count(entry.first) ? warning_counts_.at(entry.first) : 0)
        << ",\"reason\":" << json(a.reason) << '}';
  }
  out << "},\"associations\":["; first = true;
  for (const auto & rule : associations_) {
    if (!first) out << ','; first = false;
    const auto evidence = conditions_.find(rule.condition);
    out << "{\"source_group\":" << json(rule.source_group) << ",\"category\":" << json(rule.category)
        << ",\"dependent_group\":" << json(rule.dependent_group) << ",\"condition\":" << json(rule.condition)
        << ",\"required_source\":" << json(rule.source)
        << ",\"satisfied\":" << (conditionSatisfied(rule) ? "true" : "false")
        << ",\"evidence\":" << (evidence == conditions_.end() ? 0 : static_cast<int>(evidence->second.value))
        << ",\"valid_until_ns\":" << (evidence == conditions_.end() ? 0 : evidence->second.valid_until_ns) << '}';
  }
  out << "],\"events\":["; first = true;
  for (const auto & entry : events_) {
    if (!first) out << ','; first = false;
    const auto & e = entry.second;
    out << "{\"id\":" << json(e.id) << ",\"axis\":" << json(e.axis)
        << ",\"group\":" << json(e.group) << ",\"backend\":" << json(e.backend)
        << ",\"category\":" << json(e.category) << ",\"code\":" << json(e.code)
        << ",\"severity\":" << static_cast<int>(e.severity) << ",\"timestamp_ns\":" << e.timestamp_ns
        << ",\"evidence_valid\":" << (e.evidence_valid ? "true" : "false")
        << ",\"reason\":" << json(e.reason) << ",\"recovery\":" << json(e.recovery) << '}';
  }
  out << "]}";
  return out.str();
}
std::shared_ptr<Domain> domain(const std::string & name) {
  static std::mutex mutex;
  static std::map<std::string, std::shared_ptr<Domain>> domains;
  std::lock_guard<std::mutex> lock(mutex);
  auto & result = domains[name]; if (!result) result = std::make_shared<Domain>(); return result;
}
}
