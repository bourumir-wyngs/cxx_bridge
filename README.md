This is a project to bridge between Rust and ROS 2 universes, so that tools like RViz2 could be used to validate rs-opw-kinematics project that deals with path and trajectory planning for 6DOF robots with spherical wrist. While we are looking forward for Rust as a full right citizen withing ROS 2, this time seems still yeat to come.

A simple, ProtocolBuffers based format is used for messaging between Rust and ROS 2. It is described in data_format.md

The git repository contains folders "ros" (ROS project) and "rust" that is the Rust crate. The Rust crate would build and run without its ROS partner, but just one side of the bridge makes little sense. 

This crate aims for advanced users that build both ROS2 C++ and Rust projects. It is mainly intended as a development dependency of rs-opw-kinematics but may be suitable for other projects dealing with pose arrays and 6DOF robots.









