#pragma once

#include "pose_array.pb.h"
#include "joint_trajectory_dof6.pb.h"
#include "mesh.pb.h"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker.hpp>

class RosSender : public rclcpp::Node {
public:
  // Constructor: Initializes RosSender with the ROS publishers
  RosSender();

  // Sends a PoseArray message
  void sendPoseArray(const pose_array::PoseArray& poseArrayMessage);

  // Sends a Mesh message
  void sendMesh(const mesh::Mesh& pointCloudMessage);

  // Sends a PointCloud message (mesh with no triangles)
  void sendPointCloud(const mesh::Mesh& pointCloudMessage);

  // Sends a JointTrajectory message
  void sendJointTrajectory(const joint_trajectory_dof6::JointTrajectoryDof6& jointTrajectoryMessage);

private:
  // ROS publishers for PoseArray and JointTrajectory
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr m_poseArrayPublisher;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr m_trajectoryPublisher;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_pointCloudPublisher;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr m_meshPublisher;

};
