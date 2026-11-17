#include "arm_control/motion_validation.hpp"
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <Eigen/SVD>
#include <chrono>

class ServoReadiness : public rclcpp::Node {
public:
  ServoReadiness() : Node("servo_readiness", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)) {}
  void initialize() {
    if(!has_parameter("singularity_threshold"))declare_parameter("singularity_threshold",17.0);
    if(!has_parameter("joint_limit_margin"))declare_parameter("joint_limit_margin",0.1);
    threshold_=get_parameter("singularity_threshold").as_double();margin_=get_parameter("joint_limit_margin").as_double();
    if(!std::isfinite(threshold_) || threshold_<=1 || !std::isfinite(margin_) || margin_<0)
      throw std::runtime_error("Invalid Servo entry thresholds");
    robot_model_loader::RobotModelLoader loader(shared_from_this(),"robot_description",false);
    model_=loader.getModel();if(!model_ || !model_->getJointModelGroup("arm"))throw std::runtime_error("Arm model unavailable");
    sub_=create_subscription<sensor_msgs::msg::JointState>("/joint_states",10,[this](sensor_msgs::msg::JointState::ConstSharedPtr msg){
      valid_=arm_control::arm_feedback(*msg,positions_,velocities_);at_=std::chrono::steady_clock::now();
    });
    service_=create_service<std_srvs::srv::Trigger>("/arm_control/check_servo_ready",
      [this](std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr res){
        res->message="Fresh, stationary six-axis feedback required before Servo";
        if(!valid_ || std::chrono::steady_clock::now()-at_>std::chrono::milliseconds(300) || !arm_control::stationary(velocities_))return;
        moveit::core::RobotState state(model_);state.setToDefaultValues();
        for(const auto &name:arm_control::arm_names){
          const double q=positions_.at(name);const auto &b=model_->getVariableBounds(name);
          if(!b.position_bounded_ || q<b.min_position_+margin_ || q>b.max_position_-margin_){
            res->message="Servo entry too close to limit: "+name+"; plan to named ready first";return;
          }
          state.setVariablePosition(name,q);
        }
        state.update();
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(state.getJacobian(model_->getJointModelGroup("arm")));
        const auto &s=svd.singularValues();double condition=s[0]/s[s.size()-1];
        if(!std::isfinite(condition) || condition>=threshold_){
          res->message="Servo entry near singularity (condition="+std::to_string(condition)+
            "); call /arm_control/prepare_servo or plan to named ready, then retry";return;
        }
        res->success=true;res->message="Servo entry ready; condition="+std::to_string(condition);
      });
  }
private:
  double threshold_,margin_;bool valid_=false;
  arm_control::Values positions_,velocities_;
  std::chrono::steady_clock::time_point at_;
  moveit::core::RobotModelPtr model_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;
};
int main(int argc,char **argv){
  rclcpp::init(argc,argv);auto node=std::make_shared<ServoReadiness>();
  try{node->initialize();rclcpp::spin(node);}catch(const std::exception &e){RCLCPP_ERROR(node->get_logger(),"%s",e.what());rclcpp::shutdown();return 1;}
  node.reset();rclcpp::shutdown();return 0;
}
