#pragma once
#include "robot_safety/domain.hpp"
#include <atomic>
namespace robot_safety {
// Device preparation is performed on a service executor. Command buffers belong exclusively
// to the controller-manager thread. Only prepared crosses those two threads.
struct AxisSession {
  std::string group;
  uint64_t epoch = 0;
  bool permission_lost = true;
  std::shared_ptr<std::atomic<uint64_t>> prepared_epoch = std::make_shared<std::atomic<uint64_t>>(0);
  std::shared_ptr<std::atomic<bool>> prepared = std::make_shared<std::atomic<bool>>(false);
};
// Called only after a denied write: keep outputs stopped for this cycle. The
// runtime may have advanced Armed -> Running since permits() was checked; that
// valid new session must not lose preparation because of the earlier snapshot.
inline bool preparationRevoked(Domain & domain, const AxisSession & session) {
  const auto group = domain.group(session.group);
  if (group.state == Domain::State::Armed ||
      (group.state == Domain::State::Running && group.epoch == session.epoch)) return false;
  return !session.permission_lost ||
    (session.prepared->load() && (domain.globallyInhibited() ||
                                session.prepared_epoch->load() != group.epoch));
}
inline void commandFault(const std::shared_ptr<Domain> & d, const std::string & axis,
                         const std::string & group, const std::string & reason) {
  Domain::Event e; e.id = "command:" + axis + ":" + std::to_string(d->epoch(group));
  e.axis = axis; e.category = "invalid_running_output"; e.reason = reason;
  e.recovery = "repair command source, acknowledge, prepare/arm/start with a new command";
  d->event(e);
}
}
