use prost::Message; // For Protobuf serialization
use nalgebra::Isometry3; // For pose representation
use std::io::{Error, Write}; // For I/O operations
use std::net::TcpStream; // For TCP communication

/// Define the Magic Number (3-byte sequence) that acts as a unique identifier
/// for verifying the validity of client messages.
pub const MAGIC_NUMBER: [u8; 3] = [0xAA, 0x55, 0x01];

/// Message type constants as ASCII bytes
pub const POSE_ARRAY_MESSAGE: u8 = 0x01;
pub const JOINT_TRAJECTORY_MESSAGE: u8 = 0x02;

// Generated protobuf modules
mod pose_array {
    include!(concat!(env!("OUT_DIR"), "/pose_array.rs"));
}

mod joint_trajectory_dof6 {
    include!(concat!(env!("OUT_DIR"), "/joint_trajectory_dof6.rs"));
}

/// Struct to manage the sender configuration
pub struct Sender {
    host: String,
    port: u16,
}

impl Sender {
    /// Construct a new `Sender` with the specified host and port.
    pub fn new(host: &str, port: u16) -> Self {
        Self {
            host: host.to_string(),
            port,
        }
    }

    /// General function to send a message to the server
    fn send_message(
        &self,
        message_type: u8,
        message: Vec<u8>,
    ) -> Result<(), Error> {
        let mut stream = TcpStream::connect((self.host.as_str(), self.port))?;

        // Step 1: Send the magic number (3 bytes)
        stream.write_all(&MAGIC_NUMBER)?;

        // Step 2: Send the message type (1 byte)
        stream.write_all(&[message_type])?;

        // Step 3: Calculate and send the message length (4 bytes in big-endian)
        let message_length = message.len() as u32;
        stream.write_all(&message_length.to_be_bytes())?;

        // Step 4: Send the serialized message
        stream.write_all(&message)?;

        Ok(())
    }

    /// Sends a pose message, taking an `Isometry3<f64>` as the pose
    pub fn send_pose_message(&self, pose: Isometry3<f64>) -> Result<(), Error> {
        // Create a PoseArray protobuf message from the provided pose
        let message = create_pose_array_message(vec![pose]);

        // Use the general send_message function
        self.send_message(POSE_ARRAY_MESSAGE, message)
    }

    /// Sends a joint trajectory message, taking a vector of [f64; 6] as input
    pub fn send_joint_trajectory_message(
        &self,
        trajectory_steps: Vec<[f64; 6]>,
    ) -> Result<(), Error> {
        // Create a JointTrajectoryDof6 protobuf message from the provided trajectory
        let message = create_joint_trajectory_message(trajectory_steps);

        // Use the general send_message function
        self.send_message(JOINT_TRAJECTORY_MESSAGE, message)
    }
}

/// Helper function to create a PoseArray protobuf message
fn create_pose_array_message(pose_array: Vec<Isometry3<f64>>) -> Vec<u8> {
    let mut message = pose_array::PoseArray {
        topic: "pose_array".to_string(),
        frame: "world".to_string(),
        poses: Vec::new(),
    };

    // Populate the protobuf fields from the nalgebra Isometry3 vector
    for pose in pose_array {
        message.poses.push(pose_array::Pose {
            x: pose.translation.vector.x,
            y: pose.translation.vector.y,
            z: pose.translation.vector.z,
            qx: pose.rotation.i,
            qy: pose.rotation.j,
            qz: pose.rotation.k,
            qw: pose.rotation.w,
        });
    }

    // Serialize the message into a vector of bytes
    let mut buf = Vec::new();
    message.encode(&mut buf).expect("Failed to serialize PoseArray message.");
    buf
}

/// Helper function to create a JointTrajectoryDof6 protobuf message
fn create_joint_trajectory_message(trajectory_steps: Vec<[f64; 6]>) -> Vec<u8> {
    let mut message = joint_trajectory_dof6::JointTrajectoryDof6 {
        topic: "trajectory_topic".to_string(),
        steps: Vec::new(),
    };

    // Populate the protobuf fields from the [f64; 6] trajectory steps
    for step in trajectory_steps {
        message.steps.push(joint_trajectory_dof6::JointStepDof6 {
            j1: step[0] as f32,
            j2: step[1] as f32,
            j3: step[2] as f32,
            j4: step[3] as f32,
            j5: step[4] as f32,
            j6: step[5] as f32,
        });
    }

    // Serialize the message into a vector of bytes
    let mut buf = Vec::new();
    message
        .encode(&mut buf)
        .expect("Failed to serialize JointTrajectoryDof6 message.");
    buf
}