#include "cxx_bridge/protobuf_server.hpp"

// Constructor: Initializes the ROS 2 node and publishers
RosSender::RosSender()
  : rclcpp::Node("ros_sender") { // Initialize the node with name "ros_sender"
  m_poseArrayPublisher = this->create_publisher<geometry_msgs::msg::PoseArray>(
    "pose_array", 10); // Create publisher for PoseArray

  m_jointTrajectoryPublisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "joint_trajectory", 10); // Create publisher for JointTrajectory
}

// Sends a PoseArray message
void RosSender::sendPoseArray(const pose_array::PoseArray& poseArrayMessage) {
  printf("Sending PoseArray message %d poses\n", poseArrayMessage.poses_size());
  // Convert Protobuf message into a ROS 2 PoseArray message
  geometry_msgs::msg::PoseArray rosPoseArray;
  // Assume poseArrayMessage provides access to its poses.
  for (const auto& pose : poseArrayMessage.poses()) {
    geometry_msgs::msg::Pose &rosPose = rosPoseArray.poses.emplace_back();
    rosPose.position.x = pose.x();
    rosPose.position.y = pose.y();
    rosPose.position.z = pose.z();

    rosPose.orientation.x = pose.qx();
    rosPose.orientation.y = pose.qy();
    rosPose.orientation.z = pose.qz();
    rosPose.orientation.w = pose.qw();
  }
  m_poseArrayPublisher->publish(rosPoseArray); // Publish the PoseArray message
}

// Sends a JointTrajectory message
void RosSender::sendJointTrajectory(
  const joint_trajectory_dof6::JointTrajectoryDof6& jointTrajectoryMessage) {
  // Convert Protobuf JointTrajectoryDof6 message into a ROS 2 JointTrajectory message
  trajectory_msgs::msg::JointTrajectory rosJointTrajectory;
  for (const auto& step : jointTrajectoryMessage.steps()) {
    trajectory_msgs::msg::JointTrajectoryPoint rosPoint;
    // Each joint position
    rosPoint.positions = {step.j1(), step.j2(), step.j3(), step.j4(), step.j5(), step.j6()};

    rosJointTrajectory.points.push_back(rosPoint);
  }
  m_jointTrajectoryPublisher->publish(rosJointTrajectory); // Publish the JointTrajectory message
}
