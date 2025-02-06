#include "cxx_bridge/protobuf_server.hpp"

// Constructor: Initializes the ROS 2 node and publishers
RosSender::RosSender()
    : rclcpp::Node("ros_sender")
{
    // Initialize the node with name "ros_sender"
    m_poseArrayPublisher = this->create_publisher<geometry_msgs::msg::PoseArray>(
        "pose_array", 10);

    m_trajectoryPublisher = this->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
        "display_planned_path", 100);

    m_pointCloudPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "point_cloud", 10);
}

// Sends a PoseArray message
void RosSender::sendPoseArray(const pose_array::PoseArray& poseArrayMessage)
{
    printf("Sending PoseArray message %d poses\n", poseArrayMessage.poses_size());
    // Convert Protobuf message into a ROS 2 PoseArray message
    geometry_msgs::msg::PoseArray rosPoseArray;
    rosPoseArray.header.frame_id = poseArrayMessage.frame();
    rosPoseArray.header.stamp = rclcpp::Clock().now();

    // Assume poseArrayMessage provides access to its poses.
    for (const auto& pose : poseArrayMessage.poses())
    {
        geometry_msgs::msg::Pose& rosPose = rosPoseArray.poses.emplace_back();
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
    const joint_trajectory_dof6::JointTrajectoryDof6& jointTrajectoryMessage)
{
    printf("Sending DisplayTrajectory message based on joint trajectory, %d steps.\n",
           jointTrajectoryMessage.steps_size());

    // Create a DisplayTrajectory message
    moveit_msgs::msg::DisplayTrajectory displayRobotStateMsg;

    // Populate the joint names in the JointTrajectory message
    std::vector<std::string> jointNames = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

    trajectory_msgs::msg::JointTrajectory rosJointTrajectory;
    rosJointTrajectory.joint_names = jointNames;

    // Iterate through the steps in jointTrajectoryMessage
    for (const auto& step : jointTrajectoryMessage.steps())
    {
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

void RosSender::sendPointCloud(const point_cloud::PointCloud& pointCloudMessage)
{
    printf("Sending PointCloud message with %d points.\n", pointCloudMessage.points_size());

    // Create a ROS 2 PointCloud2 message
    sensor_msgs::msg::PointCloud2 rosPointCloud;

    // Set the header with the frame ID and the current timestamp
    rosPointCloud.header.frame_id = pointCloudMessage.frame();
    rosPointCloud.header.stamp = rclcpp::Clock().now();

    // Set the height (1 for an unorganized point cloud)
    rosPointCloud.height = 1;

    // Set the width to the number of points in the point cloud
    rosPointCloud.width = pointCloudMessage.points_size();

    // The point cloud is dense (i.e., all points are valid)
    rosPointCloud.is_dense = true;

    // Define the fields in the PointCloud2 message (x, y, z, and rgb)
    rosPointCloud.fields.resize(4);

    rosPointCloud.fields[0].name = "x";
    rosPointCloud.fields[0].offset = 0;
    rosPointCloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
    rosPointCloud.fields[0].count = 1;

    rosPointCloud.fields[1].name = "y";
    rosPointCloud.fields[1].offset = 4; // Offset is cumulative; x is 4 bytes (float32)
    rosPointCloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
    rosPointCloud.fields[1].count = 1;

    rosPointCloud.fields[2].name = "z";
    rosPointCloud.fields[2].offset = 8; // Offset after y (4 bytes)
    rosPointCloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
    rosPointCloud.fields[2].count = 1;

    rosPointCloud.fields[3].name = "rgb";
    rosPointCloud.fields[3].offset = 12; // Offset after z (4 bytes)
    rosPointCloud.fields[3].datatype = sensor_msgs::msg::PointField::UINT32; // RGB is packed as uint32
    rosPointCloud.fields[3].count = 1;

    // Set the size of each point in bytes (x, y, z = 3x float32 = 12 bytes, rgb = 4 bytes -> total = 16)
    rosPointCloud.point_step = 16;
    rosPointCloud.row_step = rosPointCloud.point_step * rosPointCloud.width;

    // Allocate space for the binary data
    rosPointCloud.data.resize(rosPointCloud.row_step);

    // Populate the data
    uint8_t* ptr = rosPointCloud.data.data(); // Pointer to binary data buffer

    for (const auto& point : pointCloudMessage.points())
    {
        // Position (x, y, z)
        float x = point.x();
        float y = point.y();
        float z = point.z();
        memcpy(ptr, &x, sizeof(float));
        ptr += sizeof(float);
        memcpy(ptr, &y, sizeof(float));
        ptr += sizeof(float);
        memcpy(ptr, &z, sizeof(float));
        ptr += sizeof(float);

        // RGB color (pack red, green, blue into a single uint32)
        uint32_t rgb = (pointCloudMessage.red() << 16) | (pointCloudMessage.green() << 8) | (pointCloudMessage.blue());
        memcpy(ptr, &rgb, sizeof(uint32_t));
        ptr += sizeof(uint32_t);
    }

    // Publish the PointCloud2 message
    m_pointCloudPublisher->publish(rosPointCloud);
    printf("PointCloud2 message published.\n");
}
