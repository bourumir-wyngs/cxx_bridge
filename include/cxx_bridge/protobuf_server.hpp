#pragma once

#include "pose_array.pb.h"
#include "joint_trajectory_dof6.pb.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

#include <string>
#include <string_view>

class RosSender : public rclcpp::Node {
public:
  // Constructor: Initializes RosSender with the ROS publishers
  RosSender();

  // Sends a PoseArray message
  void sendPoseArray(const pose_array::PoseArray& poseArrayMessage);

  // Sends a JointTrajectory message
  void sendJointTrajectory(const joint_trajectory_dof6::JointTrajectoryDof6& jointTrajectoryMessage);

private:
  // ROS publishers for PoseArray and JointTrajectory
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr m_poseArrayPublisher;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr m_jointTrajectoryPublisher;
};