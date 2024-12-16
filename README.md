This is a project to bridge between Rust and ROS 2 universes, so that tools like RViz2 could be used to validate rs-opw-kinematics project that deals with path and trajectory planning for 6DOF robots with spherical wrist. While we are looking forward for Rust as a full right citizen withing ROS 2, this time seems still yeat to come.

It provides the Rust sender and ROS 2 package that receives messages and posts as valid ROS messages, visualizing pose array and robot trajectory in MoveIt compatible way. 

On the Rust side, nalgebra Isometry3 is used for pose and Vec<[f64; 6]> is used for joints.
On the ROS side, geometry_msgs::msg::PoseArray is published for poses on "pose_array" topic, and moveit_msgs::msg::DisplayTrajectory is published on "display_planned_path" topic. 

Topic names are currently fixed, as well as joint names ("joint_1" to "joint_6"). The host is 127.0.0.1 and the port is 5555. From one side, it is configuration-free this way, from another, you need to modify the source code to change these settings. 

A simple, ProtocolBuffers based format is used for messaging between Rust and ROS 2. It is described in data_format.md

The git repository contains folders "ros" (ROS project) and "rust" that is the Rust crate. The Rust crate would build and run without its ROS partner, but just one side of the bridge makes little sense. 

This crate aims for advanced users that build both ROS2 C++ and Rust projects. It is mainly intended as a development dependency of rs-opw-kinematics but may be suitable for other projects dealing with pose arrays and 6DOF robots.






