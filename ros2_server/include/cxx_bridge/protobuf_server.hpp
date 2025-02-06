#pragma once

#include "pose_array.pb.h"
#include "joint_trajectory_dof6.pb.h"
#include "point_cloud.pb.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

class RosSender : public rclcpp::Node {
public:
  // Constructor: Initializes RosSender with the ROS publishers
  RosSender();

  // Sends a PoseArray message
  void sendPoseArray(const pose_array::PoseArray& poseArrayMessage);

  // Sends a PointCloud message
  void sendPointCloud(const point_cloud::PointCloud& pointCloudMessage);

  // Sends a JointTrajectory message
  void sendJointTrajectory(const joint_trajectory_dof6::JointTrajectoryDof6& jointTrajectoryMessage);

private:
  // ROS publishers for PoseArray and JointTrajectory
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr m_poseArrayPublisher;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr m_trajectoryPublisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointCloudPublisher;

};
