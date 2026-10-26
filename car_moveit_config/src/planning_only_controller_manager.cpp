#include <moveit/controller_manager/controller_manager.h>
#include <pluginlib/class_list_macros.hpp>
#include <dlfcn.h>
#include <chrono>

namespace robot24 {
// The move_group process owns planning and scene services. It must never create
// a second trajectory execution client alongside the project runtime gate.
// Humble MoveItCpp constructs a controller manager even with execution disabled;
// expose no execution handles rather than loading unused action clients.
class PlanningOnlyControllerManager : public moveit_controller_manager::MoveItControllerManager {
public:
  void initialize(const rclcpp::Node::SharedPtr & node) override {
    // Humble 2.5.10 can retain callback-group weak control blocks in NodeBase
    // after pluginlib unloads the capabilities that instantiated them. Pin the
    // already-loaded capability code until process exit; NOLOAD never loads or
    // re-registers plugins, NODELETE preserves code during Node destruction.
    // The node can retain weak timer control blocks after plugin destruction.
    if (void * self = dlopen("librobot24_planning_controller.so", RTLD_LAZY | RTLD_NOLOAD | RTLD_NODELETE)) dlclose(self);
    pin_ = node->create_wall_timer(std::chrono::milliseconds(100), [this]() {
      bool capabilities_loaded = false;
      for (const char * library : {"libmoveit_move_group_default_capabilities.so",
            "libmoveit_default_planning_request_adapter_plugins.so", "libmoveit_ompl_planner_plugin.so",
            "libmoveit_kdl_kinematics_plugin.so", "libmoveit_simple_controller_manager.so"}) {
        if (void * handle = dlopen(library, RTLD_LAZY | RTLD_NOLOAD | RTLD_NODELETE)) {
          if (std::string(library)=="libmoveit_move_group_default_capabilities.so") capabilities_loaded=true;
          dlclose(handle);
        }
      }
      if (capabilities_loaded) pin_->cancel();
    });
  }
  ~PlanningOnlyControllerManager() override { if (pin_) pin_->cancel(); }
private:
  rclcpp::TimerBase::SharedPtr pin_;
public:
  moveit_controller_manager::MoveItControllerHandlePtr getControllerHandle(const std::string &) override { return {}; }
  void getControllersList(std::vector<std::string> & names) override { names.clear(); }
  void getActiveControllers(std::vector<std::string> & names) override { names.clear(); }
  void getControllerJoints(const std::string &, std::vector<std::string> & names) override { names.clear(); }
  ControllerState getControllerState(const std::string &) override { return {}; }
  bool switchControllers(const std::vector<std::string> &, const std::vector<std::string> &) override { return false; }
};
}
PLUGINLIB_EXPORT_CLASS(robot24::PlanningOnlyControllerManager, moveit_controller_manager::MoveItControllerManager)
