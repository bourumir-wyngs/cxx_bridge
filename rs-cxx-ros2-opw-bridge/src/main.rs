use nalgebra::{Isometry3, Point3, Translation3, UnitQuaternion};
use rs_cxx_ros2_opw_bridge::sender::Sender;
use std::vec;


fn main() {
    fn generate_points(center: Point3<f32>, radius: f32) -> Vec<(f32, f32, f32)> {
        vec![
            (center.x + radius, center.y, center.z),             // Point to the right
            (center.x, center.y + radius, center.z),             // Point upward
            (center.x - radius, center.y, center.z),             // Point to the left
        ]
    }

    let sender = Sender::new("127.0.0.1", 5555);

    // Send a pose message
    let poses = vec![Isometry3::from_parts(
        Translation3::new(1.0, 2.0, 3.0),
        UnitQuaternion::identity(),
    )];

    if let Err(e) = sender.send_pose_message(&poses) {
        eprintln!("Error sending pose message: {}", e);
    }

    // Send a joint trajectory message
    let joint_trajectory = vec![
        [0.0, 1.0, 2.0, 3.0, 4.0, 5.0],
        [6.0, 7.0, 8.0, 9.0, 10.0, 11.0],
    ];

    if let Err(e) = sender.send_joint_trajectory_message(&joint_trajectory) {
        eprintln!("Error sending joint trajectory message: {}", e);
    }

    let red = (255, 0, 0);
    let green = (0, 255, 0);
    let blue = (0, 0, 255);

    // Radius for points around each center
    let radius = 0.2;
    let transparency = 0.5;

    // Generate points
    let red_points = generate_points(Point3::new(1.5, 0.0, 0.0), radius);
    let green_points = generate_points(Point3::new(0.0, 1.6, 0.0), radius);
    let blue_points = generate_points(Point3::new(0.0, 0.0, 1.7), radius);

    // Send red point cloud
    if let Err(e) = sender.send_point_cloud_message(&red_points, red, transparency) {
        eprintln!("Error sending red point cloud: {}", e);
    }

    // Send green point cloud
    if let Err(e) = sender.send_point_cloud_message(&green_points, green, transparency) {
        eprintln!("Error sending green point cloud: {}", e);
    }

    // Send blue point cloud
    if let Err(e) = sender.send_point_cloud_message(&blue_points, blue, transparency) {
        eprintln!("Error sending blue point cloud: {}", e);
    }

}
