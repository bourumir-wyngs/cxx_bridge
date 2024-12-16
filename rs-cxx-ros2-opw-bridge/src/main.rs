use nalgebra::{Isometry3, Translation3, UnitQuaternion};
use rs_cxx_ros2_opw_bridge::sender::Sender;

fn main() {
    let sender = Sender::new("127.0.0.1", 5555);

    // Send a pose message
    let pose = Isometry3::from_parts(
        Translation3::new(1.0, 2.0, 3.0),
        UnitQuaternion::identity(),
    );

    if let Err(e) = sender.send_pose_message(pose) {
        eprintln!("Error sending pose message: {}", e);
    }

    // Send a joint trajectory message
    let joint_trajectory = vec![
        [0.0, 1.0, 2.0, 3.0, 4.0, 5.0],
        [6.0, 7.0, 8.0, 9.0, 10.0, 11.0],
    ];

    if let Err(e) = sender.send_joint_trajectory_message(joint_trajectory) {
        eprintln!("Error sending joint trajectory message: {}", e);
    }
}