#include "cxx_bridge/protobuf_server.hpp"


// Constructor: Initializes the ROS 2 node and publishers
RosSender::RosSender()
  : rclcpp::Node("ros_sender") { // Initialize the node with name "ros_sender"
  m_poseArrayPublisher = this->create_publisher<geometry_msgs::msg::PoseArray>(
    "pose_array", 10); // Create publisher for PoseArray

  m_trajectoryPublisher = this->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
    "display_planned_path", 100); // Create publisher for JointTrajectory
}

// Sends a PoseArray message
void RosSender::sendPoseArray(const pose_array::PoseArray &poseArrayMessage) {
  printf("Sending PoseArray message %d poses\n", poseArrayMessage.poses_size());
  // Convert Protobuf message into a ROS 2 PoseArray message
  geometry_msgs::msg::PoseArray rosPoseArray;
  rosPoseArray.header.frame_id = poseArrayMessage.frame();
  rosPoseArray.header.stamp = rclcpp::Clock().now();

  // Assume poseArrayMessage provides access to its poses.
  for (const auto &pose: poseArrayMessage.poses()) {
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

void RosSender::sendJointTrajectory(
  const joint_trajectory_dof6::JointTrajectoryDof6 &jointTrajectoryMessage) {

  printf("Sending DisplayTrajectory message based on joint trajectory, %d steps.\n", jointTrajectoryMessage.steps_size() );

  // Create a DisplayTrajectory message
  moveit_msgs::msg::DisplayTrajectory displayRobotStateMsg;

  // Populate the joint names in the JointTrajectory message
  std::vector<std::string> jointNames = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

  trajectory_msgs::msg::JointTrajectory rosJointTrajectory;
  rosJointTrajectory.joint_names = jointNames;

  // Iterate through the steps in jointTrajectoryMessage
  for (const auto &step : jointTrajectoryMessage.steps()) {
    trajectory_msgs::msg::JointTrajectoryPoint rosPoint;

    // Extract joint positions from the current step
    rosPoint.positions = {step.j1(), step.j2(), step.j3(), step.j4(), step.j5(), step.j6()};

    // Add the ROS-compatible joint trajectory point to the trajectory
    rosJointTrajectory.points.push_back(rosPoint);
  }

  // Wrap the JointTrajectory into a RobotTrajectory
  moveit_msgs::msg::RobotTrajectory robotTrajectory;
  robotTrajectory.joint_trajectory = rosJointTrajectory;

  // Assign RobotTrajectory to the DisplayTrajectory message
  displayRobotStateMsg.trajectory.push_back(robotTrajectory);

  // Publish the DisplayTrajectory message
  m_trajectoryPublisher->publish(displayRobotStateMsg);

  printf("Trajectory message published.\n");
}