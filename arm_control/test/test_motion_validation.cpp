#include <gtest/gtest.h>
#include "arm_control/motion_validation.hpp"
using namespace arm_control;
TEST(Feedback, IndependentGroupAndInvalidArm) {
 sensor_msgs::msg::JointState msg;msg.name=arm_names;msg.position.assign(6,0.);msg.velocity.assign(6,0.);
 msg.name.push_back("left_front_arm_joint");msg.position.push_back(NAN);msg.velocity.push_back(NAN);
 Values q,v;ASSERT_TRUE(arm_feedback(msg,q,v));EXPECT_EQ(q.size(),6u);
 msg.position[2]=NAN;EXPECT_FALSE(arm_feedback(msg,q,v));msg.position[2]=0.;
 msg.name[1]=msg.name[0];EXPECT_FALSE(arm_feedback(msg,q,v));
 msg.name=arm_names;msg.position.resize(6);msg.velocity.clear();EXPECT_FALSE(arm_feedback(msg,q,v));
}
TEST(PlanningStart, DriftAndMultiTurnAndVelocity) {
 Values q,v;for(auto &n:arm_names){q[n]=0.;v[n]=0.;}
 std::vector<double> start(6,0.);ASSERT_TRUE(matches_start(arm_names,start,q));
 q[arm_names[0]]=.011;EXPECT_FALSE(matches_start(arm_names,start,q));q[arm_names[0]]=0.;
 q[arm_names[5]]=2*M_PI;EXPECT_FALSE(matches_start(arm_names,start,q));
 EXPECT_TRUE(stationary(v));v[arm_names[2]]=.051;EXPECT_FALSE(stationary(v));
 start.pop_back();EXPECT_FALSE(matches_start(arm_names,start,q));
}
TEST(Trajectory, BoundsMalformedAndVelocity) {
 trajectory_msgs::msg::JointTrajectory t;t.joint_names=arm_names;
 trajectory_msgs::msg::JointTrajectoryPoint p;p.positions.assign(6,0.);p.velocities.assign(6,0.);t.points={p};
 std::map<std::string,Limits> limits;for(auto &n:arm_names)limits[n]={-1.,1.,1.57};
 Values measured;for(auto &n:arm_names)measured[n]=0.;
 ASSERT_TRUE(within_limits(measured,limits));measured[arm_names[0]]=1.001;EXPECT_FALSE(within_limits(measured,limits));
 ASSERT_TRUE(valid_trajectory(t,limits));t.points[0].positions[0]=1.001;EXPECT_FALSE(valid_trajectory(t,limits));
 t.points[0].positions[0]=0.;t.points[0].velocities[0]=1.58;EXPECT_FALSE(valid_trajectory(t,limits));
 t.points[0].velocities[0]=0.;t.points.push_back(t.points[0]);EXPECT_FALSE(valid_trajectory(t,limits));
 t.points.pop_back();t.joint_names[1]=t.joint_names[0];EXPECT_FALSE(valid_trajectory(t,limits));
}
