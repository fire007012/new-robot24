#pragma once
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <set>
#include <functional>
#include <utility>
namespace robot_safety {
class Domain {
public:
  enum class State { Stopped, Armed, Running, Fault };
  // The legacy whole-vehicle API remains conservative during backend migration.
  // Group methods never grant permission through the legacy permits(epoch) gate.
  enum class Severity { Reject, Warn, GroupStop, GlobalStop };
  enum class Evidence { Unknown, Satisfied, Unsatisfied };
  struct Event {
    std::string id, axis, group, backend, category, code, reason, recovery;
    Severity severity = Severity::GroupStop;
    int64_t timestamp_ns = 0;
    bool evidence_valid = false;
  };
  struct Axis {
    std::string group, backend;
    bool ready = false;
    std::string reason = "not reported";
  };
  struct Group {
    State state = State::Stopped;
    uint64_t epoch = 0;
    bool ready = false;
    std::string reason = "stopped";
    std::vector<std::string> blockers;
  };
  struct Association {
    std::string source_group, category, dependent_group, condition, source;
  };
  struct Condition {
    Evidence value = Evidence::Unknown;
    std::string source;
    int64_t valid_until_ns = 0;
  };
  using Clock = std::function<int64_t()>;
  explicit Domain(Clock clock = {});
  // Configuration is atomic and rejected while any group is Armed/Running.
  void configure(const std::map<std::string, Axis> & axes,
                 const std::vector<Association> & associations, const std::string & version);
  void unprepareAxis(const std::string & axis, const std::string & reason);
  void reportAxis(const std::string & axis, bool ready, const std::string & reason = "",
                  const std::string & code = "", const std::string & category = "feedback");
  void warning(const std::string & axis, bool active, const std::string & reason);
  void reportBackend(const std::string & backend, bool ready, const std::string & reason = "");
  void condition(const std::string & name, Evidence value, const std::string & source,
                 int64_t valid_until_ns);
  void event(const Event & event);
  void acknowledge(const std::string & id);
  bool arm(const std::string & group);
  bool run(const std::string & group);
  void halt(const std::string & group);
  bool permits(const std::string & group, uint64_t epoch) const;
  uint64_t epoch(const std::string & group) const;
  Group group(const std::string & name) const;
  std::map<std::string, Group> groups() const;
  void globalStop(const std::string & reason);
  bool resetGlobalStop();
  bool globallyInhibited() const;
  void requireOwnerLease();
  void renewOwner(int64_t lifetime_ns, int64_t issued_ns = 0);
  std::vector<Event> events() const;
  std::string axisGroup(const std::string & axis, const std::string & backend) const;
  bool grouped() const;
  void registerWriter(const std::string & backend, bool grouped);
  void invalidate(const std::string & group);
  void poll();  // Expired evidence must revoke permission even without a new device event.
  void report(const std::string & component, bool ready);
  void fault(const std::string & reason);
  bool arm();
  bool run();
  void halt();
  bool permits(uint64_t epoch) const;
  uint64_t epoch() const;
  State state() const;
  std::string reason() const;
  void observePeriod(int64_t nanoseconds);
  std::string diagnostics() const;
private:
  void expireOwner() const;
  bool ownerFresh() const;
  void refreshGroups();
  void propagate() const;
  std::map<std::string, std::pair<std::set<std::string>, std::string>> restrictions() const;
  std::set<std::string> scope(const Event & event) const;
  bool conditionSatisfied(const Association & rule) const;
  void revoke(const std::string & group, const std::string & reason) const;
  Group snapshot(const std::string & group) const;
  Clock clock_;
  std::map<std::string, Axis> axes_;
  std::map<std::string, bool> writers_;
  mutable std::map<std::string, Group> groups_;
  std::vector<Association> associations_;
  std::map<std::string, Condition> conditions_;
  std::map<std::string, Event> events_;
  std::map<std::string, uint64_t> warning_counts_;
  std::string configuration_version_;
  mutable bool global_inhibit_ = false;
  mutable std::string global_reason_;
  bool owner_required_ = false;
  int64_t owner_deadline_ = 0;
  mutable bool owner_expired_ = false;
  mutable std::mutex mutex_;
  std::map<std::string, bool> ready_{{"RobotCanopenSystem", false}, {"CanDriverSystem", false}};
  State state_ = State::Stopped;
  uint64_t epoch_ = 0;
  std::string reason_ = "stopped";
  uint64_t samples_ = 0, sum_ns_ = 0, max_ns_ = 0, min_ns_ = UINT64_MAX, late_ = 0;
};
std::shared_ptr<Domain> domain(const std::string & name);
}
