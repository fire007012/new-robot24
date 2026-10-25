#include <gtest/gtest.h>

#include <hardware_interface/system_interface.hpp>
#include <pluginlib/class_loader.hpp>

TEST(RobotCanopenPlugin, RobotSystemCanBeConstructed)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");
  ASSERT_TRUE(loader.isClassAvailable("canopen_ros2_control/RobotSystem"));
  auto system = loader.createSharedInstance("canopen_ros2_control/RobotSystem");
  ASSERT_NE(system, nullptr);
}
