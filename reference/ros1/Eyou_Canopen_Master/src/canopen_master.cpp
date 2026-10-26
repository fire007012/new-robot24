#include "canopen_hw/canopen_master.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <thread>

#include "canopen_hw/cia402_defs.hpp"
#include "canopen_hw/dcf_path_utils.hpp"
#include "canopen_hw/logging.hpp"

namespace canopen_hw {
namespace {

std::string MakeRuntimeMasterDcfPath(const std::string& source_dcf_path) {
  const auto source = std::filesystem::path(source_dcf_path);
  const auto stamp =
      std::chrono::steady_clock::now().time_since_epoch().count();
  return (std::filesystem::temp_directory_path() /
          (source.stem().string() + ".runtime." + std::to_string(stamp) +
           source.extension().string()))
      .string();
}

bool WriteTextFile(const std::string& path, const std::string& text,
                   std::string* error) {
  std::ofstream ofs(path, std::ios::trunc);
  if (!ofs.is_open()) {
    if (error) {
      *error = "open temp dcf failed: " + path;
    }
    return false;
  }

  ofs << text;
  if (!ofs.good()) {
    if (error) {
      *error = "write temp dcf failed: " + path;
    }
    return false;
  }
  return true;
}

void CleanupRuntimeMasterDcf(std::string* path) {
  if (!path || path->empty()) {
    return;
  }
  std::error_code ec;
  std::filesystem::remove(*path, ec);
  path->clear();
}

}  // namespace

CanopenMaster::CanopenMaster(const CanopenMasterConfig& config,
                             SharedState* shared_state)
    : config_(config), shared_state_(shared_state) {
  if (!config_.joints.empty()) {
    config_.axis_count = config_.joints.size();
  }
  if (config_.axis_count == 0) {
    config_.axis_count = 1;
  }
  if (config_.axis_count > SharedState::kMaxAxisCount) {
    CANOPEN_LOG_WARN("axis_count {} exceeds max {}, clamping",
                     config_.axis_count, SharedState::kMaxAxisCount);
    config_.axis_count = SharedState::kMaxAxisCount;
  }
  config_.joints.resize(config_.axis_count);
  for (std::size_t i = 0; i < config_.axis_count; ++i) {
    const uint8_t id = config_.joints[i].node_id;
    if (id == 0 || id > 127) {
      if (id > 127) {
        CANOPEN_LOG_WARN("joints[{}].node_id={} out of range 1..127, replacing with default {}",
                         i, static_cast<int>(id), (i + 1));
      }
      config_.joints[i].node_id = static_cast<uint8_t>(i + 1);
    }
  }
  // 预分配驱动容器容量，保证运行阶段不会因为扩容触发堆分配。
  axis_drivers_.reserve(config_.axis_count);
}

bool CanopenMaster::Start() {
  if (running_.load()) {
    return true;
  }

  if (config_.master_dcf_path.empty()) {
    CANOPEN_LOG_ERROR("CanopenMaster start failed: master_dcf_path is empty");
    return false;
  }
  if (!std::filesystem::exists(config_.master_dcf_path)) {
    CANOPEN_LOG_ERROR("CanopenMaster start failed: master DCF not found: {}",
                      config_.master_dcf_path);
    return false;
  }

  CleanupRuntimeMasterDcf(&runtime_master_dcf_path_);
  runtime_master_dcf_path_ = config_.master_dcf_path;

  std::string rewritten_dcf;
  std::vector<DcfUploadFileResolution> upload_files;
  std::string rewrite_error;
  if (!RewriteMasterDcfUploadPaths(config_.master_dcf_path, &rewritten_dcf,
                                   &upload_files, &rewrite_error)) {
    CANOPEN_LOG_ERROR("CanopenMaster start failed: {}", rewrite_error);
    return false;
  }

  std::vector<std::string> missing_upload_files;
  bool needs_runtime_copy = false;
  for (const auto& upload_file : upload_files) {
    if (upload_file.original_path != upload_file.resolved_path) {
      needs_runtime_copy = true;
    }
    if (!upload_file.exists) {
      missing_upload_files.push_back(upload_file.resolved_path);
    }
  }

  if (!missing_upload_files.empty()) {
    std::ostringstream oss;
    for (std::size_t i = 0; i < missing_upload_files.size(); ++i) {
      if (i > 0) {
        oss << ", ";
      }
      oss << missing_upload_files[i];
    }
    CANOPEN_LOG_ERROR(
        "CanopenMaster start failed: UploadFile target(s) missing for {}: {}",
        config_.master_dcf_path, oss.str());
    return false;
  }

  if (needs_runtime_copy) {
    runtime_master_dcf_path_ = MakeRuntimeMasterDcfPath(config_.master_dcf_path);
    if (!WriteTextFile(runtime_master_dcf_path_, rewritten_dcf,
                       &rewrite_error)) {
      CANOPEN_LOG_ERROR("CanopenMaster start failed: {}", rewrite_error);
      CleanupRuntimeMasterDcf(&runtime_master_dcf_path_);
      return false;
    }
    // TODO(rera): move UploadFile path normalization fully into dcf generation,
    // then this runtime temp DCF can be removed.
    CANOPEN_LOG_INFO("Prepared runtime master DCF with resolved UploadFile paths: {}",
                     runtime_master_dcf_path_);
  }

  try {
    io_guard_ = std::make_unique<lely::io::IoGuard>();
    io_ctx_ = std::make_unique<lely::io::Context>();
    io_poll_ = std::make_unique<lely::io::Poll>(*io_ctx_);
    ev_loop_ = std::make_unique<lely::ev::Loop>(io_poll_->get_poll());
    io_timer_ = std::make_unique<lely::io::Timer>(
        *io_poll_, ev_loop_->get_executor(), CLOCK_MONOTONIC);
    can_ctrl_ =
        std::make_unique<lely::io::CanController>(config_.can_interface.c_str());
    can_chan_ =
        std::make_unique<lely::io::CanChannel>(*io_poll_,
                                               ev_loop_->get_executor());
    can_chan_->open(*can_ctrl_);

    master_ = std::make_unique<lely::canopen::AsyncMaster>(
        ev_loop_->get_executor(), *io_timer_, *can_chan_,
        runtime_master_dcf_path_, std::string(), config_.master_node_id);
    CreateAxisDrivers(*master_);

    ev_loop_->restart();
    ev_thread_ = std::thread([this]() { ev_loop_->run(); });
    master_->Reset();

    running_.store(true);
    return true;
  } catch (const std::exception& e) {
    CANOPEN_LOG_ERROR("CanopenMaster start failed: {}", e.what());
    if (ev_loop_) {
      ev_loop_->stop();
    }
    if (ev_thread_.joinable()) {
      ev_thread_.join();
    }
    axis_drivers_.clear();
    master_.reset();
    can_chan_.reset();
    can_ctrl_.reset();
    io_timer_.reset();
    ev_loop_.reset();
    io_poll_.reset();
    io_ctx_.reset();
    io_guard_.reset();
    CleanupRuntimeMasterDcf(&runtime_master_dcf_path_);
    running_.store(false);
    return false;
  }
}

void CanopenMaster::Stop() {
  if (!running_.load() && !master_ && !ev_loop_ && axis_drivers_.empty()) {
    return;
  }

  GracefulShutdown();

  if (ev_loop_) {
    ev_loop_->stop();
  }
  if (ev_thread_.joinable()) {
    ev_thread_.join();
  }

  axis_drivers_.clear();
  master_.reset();
  can_chan_.reset();
  can_ctrl_.reset();
  io_timer_.reset();
  ev_loop_.reset();
  io_poll_.reset();
  io_ctx_.reset();
  io_guard_.reset();
  CleanupRuntimeMasterDcf(&runtime_master_dcf_path_);

  running_.store(false);
}

bool CanopenMaster::GracefulShutdown(std::string* detail) {
  if (detail) {
    detail->clear();
  }
  if (axis_drivers_.empty()) {
    return true;
  }

  bool all_ok = true;
  auto append_pending = [&](const char* phase, const std::vector<std::size_t>& pending) {
    if (!detail || pending.empty()) {
      return;
    }
    std::ostringstream oss;
    oss << phase << " timeout; pending axes: ";
    for (std::size_t i = 0; i < pending.size(); ++i) {
      const std::size_t axis_index = pending[i];
      if (i > 0) {
        oss << ", ";
      }
      oss << axis_index;
      if (axis_index < config_.joints.size()) {
        oss << "(node=" << static_cast<int>(config_.joints[axis_index].node_id)
            << ")";
      }
    }
    if (!detail->empty()) {
      *detail += "; ";
    }
    *detail += oss.str();
  };

  // 注意: Stop() 必须在 Lely 事件循环仍在运行时调用，否则 RPDO 不再更新，
  // 下面的等待将只会超时返回。
  for (const auto& axis : axis_drivers_) {
    if (axis) {
      // 先同步状态机请求，避免周期控制字路径继续拉回使能。
      axis->RequestDisable();
      axis->SendControlword(kCtrl_DisableOperation);
    }
  }

  std::vector<std::size_t> pending;
  if (!WaitForAllState(CiA402State::SwitchedOn,
                       std::chrono::steady_clock::now() +
                           std::chrono::milliseconds(2000),
                       &pending)) {
    all_ok = false;
    append_pending("disable->switched_on", pending);
  }

  for (const auto& axis : axis_drivers_) {
    if (axis) {
      axis->SendControlword(kCtrl_Shutdown);
    }
  }

  if (!WaitForAllState(CiA402State::ReadyToSwitchOn,
                       std::chrono::steady_clock::now() +
                           std::chrono::milliseconds(1000),
                       &pending)) {
    all_ok = false;
    append_pending("shutdown->ready_to_switch_on", pending);
  }

  if (axis_drivers_.front()) {
    axis_drivers_.front()->SendNmtStopAll();
  }

  return all_ok;
}

bool CanopenMaster::WaitForAllState(
    CiA402State target_state,
    std::chrono::steady_clock::time_point deadline,
    std::vector<std::size_t>* pending_axes) {
  std::vector<std::size_t> pending;

  while (std::chrono::steady_clock::now() < deadline) {
    pending.clear();
    for (std::size_t i = 0; i < axis_drivers_.size(); ++i) {
      const auto& axis = axis_drivers_[i];
      if (!axis) {
        continue;
      }
      if (axis->feedback_state() != target_state) {
        pending.emplace_back(i);
      }
    }
    if (pending.empty()) {
      if (pending_axes) {
        pending_axes->clear();
      }
      return true;
    }

    if (shared_state_) {
      // 阻塞等待反馈更新通知，而非固定 sleep 轮询。
      // 每次 UpdateFeedback() 完成后会唤醒，避免空转消耗 CPU。
      shared_state_->WaitForStateChange(
          std::min(deadline,
                   std::chrono::steady_clock::now() +
                       std::chrono::milliseconds(10)));
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  if (pending_axes) {
    pending_axes->clear();
    for (std::size_t i = 0; i < axis_drivers_.size(); ++i) {
      const auto& axis = axis_drivers_[i];
      if (!axis) {
        continue;
      }
      if (axis->feedback_state() != target_state) {
        pending_axes->emplace_back(i);
      }
    }
  }
  return false;
}

void CanopenMaster::CreateAxisDrivers(lely::canopen::BasicMaster& can_master) {
  // 初始化阶段函数: clear 后重新 emplace，不会超过预留容量。
  axis_drivers_.clear();

  for (std::size_t i = 0; i < config_.axis_count; ++i) {
    const auto& joint_cfg = config_.joints[i];
    const uint8_t node_id = joint_cfg.node_id;
    auto axis = std::make_unique<AxisDriver>(can_master, node_id, i, shared_state_,
                                             joint_cfg.verify_pdo_mapping,
                                             config_.master_dcf_path,
                                             joint_cfg.ip_interpolation_period_ms,
                                             joint_cfg.default_mode);
    axis->ConfigureStateMachine(joint_cfg.position_lock_threshold,
                                joint_cfg.max_fault_resets,
                                joint_cfg.fault_reset_hold_cycles);
    axis_drivers_.emplace_back(std::move(axis));
  }
}

bool CanopenMaster::EnableAxis(std::size_t axis_index) {
  if (axis_index >= axis_drivers_.size() || !axis_drivers_[axis_index]) {
    return false;
  }
  axis_drivers_[axis_index]->RequestEnable();
  return true;
}

bool CanopenMaster::EnableAll() {
  if (!running_.load() || axis_drivers_.empty()) {
    return false;
  }

  bool ok = true;
  for (const auto& axis : axis_drivers_) {
    if (!axis) {
      ok = false;
      continue;
    }
    axis->RequestEnable();
  }
  return ok;
}

bool CanopenMaster::DisableAxis(std::size_t axis_index) {
  if (axis_index >= axis_drivers_.size() || !axis_drivers_[axis_index]) {
    return false;
  }
  axis_drivers_[axis_index]->RequestDisable();
  return true;
}

bool CanopenMaster::ResetAxisFault(std::size_t axis_index) {
  if (axis_index >= axis_drivers_.size() || !axis_drivers_[axis_index]) {
    return false;
  }
  axis_drivers_[axis_index]->ResetFault();
  // 保持历史语义：手动单轴复位后立即请求使能。
  axis_drivers_[axis_index]->RequestEnable();
  return true;
}

bool CanopenMaster::HaltAll() {
  if (!running_.load() || axis_drivers_.empty()) {
    return false;
  }

  bool ok = true;
  for (const auto& axis : axis_drivers_) {
    if (!axis) {
      ok = false;
      continue;
    }
    axis->RequestEnable();
    axis->RequestHalt();
  }
  return ok;
}

bool CanopenMaster::ResumeAll() {
  if (!running_.load() || axis_drivers_.empty()) {
    return false;
  }

  bool ok = true;
  for (const auto& axis : axis_drivers_) {
    if (!axis) {
      ok = false;
      continue;
    }
    axis->RequestResume();
    axis->RequestEnable();
  }
  return ok;
}

bool CanopenMaster::RecoverFaultedAxes(std::string* detail) {
  return ResetAllFaults(detail);
}

bool CanopenMaster::ResetAllFaults(std::string* detail) {
  if (detail) {
    detail->clear();
  }
  if (!running_.load() || axis_drivers_.empty()) {
    if (detail) {
      *detail = "master not running";
    }
    return false;
  }

  const auto is_fault_state = [](CiA402State st) {
    return st == CiA402State::Fault || st == CiA402State::FaultReactionActive;
  };

  std::vector<std::size_t> fault_axes;
  std::vector<std::size_t> latched_fault_axes;
  SharedSnapshot initial_snapshot;
  if (shared_state_) {
    initial_snapshot = shared_state_->Snapshot();
  }

  for (std::size_t i = 0; i < axis_drivers_.size(); ++i) {
    const auto& axis = axis_drivers_[i];
    if (!axis) {
      continue;
    }

    const CiA402State st = axis->feedback_state();
    const bool state_fault = is_fault_state(st);
    const bool latched_fault =
        shared_state_ && i < initial_snapshot.feedback.size()
            ? initial_snapshot.feedback[i].is_fault
            : false;

    if (state_fault) {
      fault_axes.emplace_back(i);
    } else if (latched_fault) {
      // 历史闩锁故障：实际状态已脱离 Fault/FRA，只需清缓存闩锁。
      latched_fault_axes.emplace_back(i);
    }
  }

  if (fault_axes.empty()) {
    for (const std::size_t axis_index : latched_fault_axes) {
      if (axis_index < axis_drivers_.size() && axis_drivers_[axis_index]) {
        axis_drivers_[axis_index]->ResetFault();
      }
    }
    if (detail) {
      *detail =
          latched_fault_axes.empty() ? "no faulted axis" : "fault latch cleared";
    }
    return true;
  }

  for (const std::size_t axis_index : fault_axes) {
    const auto& axis = axis_drivers_[axis_index];
    if (!axis) {
      continue;
    }
    axis->RequestDisable();
    axis->SendControlword(kCtrl_DisableVoltage);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  for (const std::size_t axis_index : fault_axes) {
    const auto& axis = axis_drivers_[axis_index];
    if (!axis) {
      continue;
    }
    axis->SendControlword(kCtrl_FaultReset);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(5));

  for (const std::size_t axis_index : fault_axes) {
    const auto& axis = axis_drivers_[axis_index];
    if (!axis) {
      continue;
    }
    axis->SendControlword(kCtrl_DisableVoltage);
  }

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(1500);
  std::vector<std::size_t> pending;
  while (std::chrono::steady_clock::now() < deadline) {
    pending.clear();

    for (const std::size_t axis_index : fault_axes) {
      bool still_fault = false;
      if (axis_index < axis_drivers_.size() && axis_drivers_[axis_index]) {
        const CiA402State st = axis_drivers_[axis_index]->feedback_state();
        still_fault = is_fault_state(st);
      }
      if (still_fault) {
        pending.emplace_back(axis_index);
      }
    }

    if (pending.empty()) {
      for (const std::size_t axis_index : fault_axes) {
        if (axis_index < axis_drivers_.size() && axis_drivers_[axis_index]) {
          axis_drivers_[axis_index]->ResetFault();
        }
      }
      for (const std::size_t axis_index : latched_fault_axes) {
        if (axis_index < axis_drivers_.size() && axis_drivers_[axis_index]) {
          axis_drivers_[axis_index]->ResetFault();
        }
      }
      return true;
    }

    if (shared_state_) {
      shared_state_->WaitForStateChange(
          std::min(deadline,
                   std::chrono::steady_clock::now() +
                       std::chrono::milliseconds(20)));
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  if (detail) {
    std::ostringstream oss;
    oss << "fault reset failed for axes: ";
    for (std::size_t i = 0; i < pending.size(); ++i) {
      if (i > 0) {
        oss << ", ";
      }
      oss << pending[i];
      if (pending[i] < config_.joints.size()) {
        oss << "(node=" << static_cast<int>(config_.joints[pending[i]].node_id)
            << ")";
      }
    }
    oss << "; try /shutdown then /init to reinitialize";
    *detail = oss.str();
  }
  return false;
}

void CanopenMaster::EmergencyStop() {
  for (const auto& axis : axis_drivers_) {
    if (axis) {
      axis->RequestDisable();
      axis->SendControlword(kCtrl_DisableVoltage);
    }
  }
}

bool CanopenMaster::GetAxisFeedback(std::size_t axis_index,
                                    AxisFeedback* out) const {
  if (!out || !shared_state_ || axis_index >= axis_drivers_.size()) {
    return false;
  }
  const SharedSnapshot snap = shared_state_->Snapshot();
  if (axis_index >= snap.feedback.size()) {
    return false;
  }
  *out = snap.feedback[axis_index];
  return true;
}

const HealthCounters* CanopenMaster::GetHealthCounters(
    std::size_t axis_index) const {
  if (axis_index >= axis_drivers_.size() || !axis_drivers_[axis_index]) {
    return nullptr;
  }
  return &axis_drivers_[axis_index]->health();
}

bool CanopenMaster::WaitForSdoIdle(std::size_t axis_index,
                                   std::chrono::milliseconds timeout) const {
  if (axis_index >= axis_drivers_.size() || !axis_drivers_[axis_index]) {
    return false;
  }
  return axis_drivers_[axis_index]->WaitForSdoIdle(timeout);
}

bool CanopenMaster::WaitForStartupComplete(
    std::size_t axis_index, std::chrono::milliseconds timeout) const {
  if (axis_index >= axis_drivers_.size() || !axis_drivers_[axis_index]) {
    return false;
  }
  return axis_drivers_[axis_index]->WaitForStartupComplete(timeout);
}

bool CanopenMaster::WaitForAllSdoIdle(
    std::chrono::milliseconds timeout,
    std::vector<std::size_t>* pending_axes) const {
  auto collect_pending = [this, pending_axes]() {
    if (pending_axes) {
      pending_axes->clear();
    }

    bool all_idle = true;
    for (std::size_t i = 0; i < axis_drivers_.size(); ++i) {
      const auto& axis = axis_drivers_[i];
      if (!axis) {
        continue;
      }
      if (!axis->sdo_idle()) {
        all_idle = false;
        if (pending_axes) {
          pending_axes->push_back(i);
        }
      }
    }
    return all_idle;
  };

  if (collect_pending()) {
    return true;
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (collect_pending()) {
      return true;
    }
  }

  return collect_pending();
}

bool CanopenMaster::WaitForAllStartupComplete(
    std::chrono::milliseconds timeout,
    std::vector<std::size_t>* pending_axes) const {
  auto collect_pending = [this, pending_axes]() {
    if (pending_axes) {
      pending_axes->clear();
    }

    bool all_ready = true;
    for (std::size_t i = 0; i < axis_drivers_.size(); ++i) {
      const auto& axis = axis_drivers_[i];
      if (!axis) {
        continue;
      }
      if (!axis->startup_complete()) {
        all_ready = false;
        if (pending_axes) {
          pending_axes->push_back(i);
        }
      }
    }
    return all_ready;
  };

  if (collect_pending()) {
    return true;
  }

  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if (collect_pending()) {
      return true;
    }
  }

  return collect_pending();
}

AxisDriver* CanopenMaster::FindDriverByNodeId(uint8_t node_id) {
  for (std::size_t i = 0; i < config_.joints.size() && i < axis_drivers_.size();
       ++i) {
    if (config_.joints[i].node_id == node_id && axis_drivers_[i]) {
      return axis_drivers_[i].get();
    }
  }
  return nullptr;
}

}  // namespace canopen_hw
