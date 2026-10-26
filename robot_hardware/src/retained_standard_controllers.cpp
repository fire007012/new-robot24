#include "robot_hardware/plugin_lifetime.hpp"
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "joint_state_broadcaster/joint_state_broadcaster.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace robot_hardware {
// Humble's executor can outlive controller_manager's plugin loaders. Its weak
// callback-group entries still contain deleters instantiated in these upstream
// DSOs. Retaining this wrapper also retains its linked upstream dependencies;
// all controller behavior and normal object destruction stay upstream-owned.
class RetainedJointTrajectoryController
  : public joint_trajectory_controller::JointTrajectoryController {
public:
  controller_interface::CallbackReturn on_init() override {
    retainPluginCode();
    return JointTrajectoryController::on_init();
  }
};

class RetainedJointStateBroadcaster
  : public joint_state_broadcaster::JointStateBroadcaster {
public:
  controller_interface::CallbackReturn on_init() override {
    retainPluginCode();
    return JointStateBroadcaster::on_init();
  }
};
}

PLUGINLIB_EXPORT_CLASS(robot_hardware::RetainedJointTrajectoryController,
                      controller_interface::ControllerInterface)
PLUGINLIB_EXPORT_CLASS(robot_hardware::RetainedJointStateBroadcaster,
                      controller_interface::ControllerInterface)
