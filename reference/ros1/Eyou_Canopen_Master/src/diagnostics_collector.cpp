#include "canopen_hw/diagnostics_collector.hpp"

#include "canopen_hw/canopen_master.hpp"
#include "canopen_hw/health_counters.hpp"

namespace canopen_hw {

DiagnosticsCollector::DiagnosticsCollector(CanopenMaster* master)
    : master_(master) {}

SystemDiagnostics DiagnosticsCollector::Collect() const {
  SystemDiagnostics diag;

  if (!master_) {
    return diag;
  }

  diag.master_running = master_->running();
  diag.axis_count = master_->axis_count();

  const auto& config = master_->config();
  bool all_op = true;

  for (std::size_t i = 0; i < diag.axis_count; ++i) {
    AxisDiagnostics ad;

    if (i < config.joints.size()) {
      ad.name = config.joints[i].name;
      ad.node_id = config.joints[i].node_id;
    }

    AxisFeedback fb;
    if (master_->GetAxisFeedback(i, &fb)) {
      ad.state = fb.state;
      ad.is_operational = fb.is_operational;
      ad.is_fault = fb.is_fault;
      ad.heartbeat_lost = fb.heartbeat_lost;
      ad.last_emcy_eec = fb.last_emcy_eec;
      if (!fb.is_operational) {
        all_op = false;
      }
    } else {
      all_op = false;
    }

    const HealthCounters* hc = master_->GetHealthCounters(i);
    if (hc) {
      ad.emcy_count = hc->emcy_count.load(std::memory_order_relaxed);
      ad.heartbeat_lost_count = hc->heartbeat_lost.load(std::memory_order_relaxed);
      ad.heartbeat_recovered_count = hc->heartbeat_recovered.load(std::memory_order_relaxed);
      ad.fault_reset_attempts = hc->fault_reset_attempts.load(std::memory_order_relaxed);
      ad.boot_retries = hc->boot_retries.load(std::memory_order_relaxed);
    }

    diag.axes.push_back(std::move(ad));
  }

  diag.all_operational = all_op && diag.axis_count > 0;
  return diag;
}

}  // namespace canopen_hw
