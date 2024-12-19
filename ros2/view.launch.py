from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import Shutdown 

def generate_launch_description():
    urdf_path = 'goblet.urdf'
    srdf_path = 'goblet.srdf'

    return LaunchDescription([
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=['joint_state_publisher_params.yaml']            
        ),
        Node(
            package='robot_state_publisher',
            name='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': open(urdf_path).read()}]
        ),
        Node(
            package='moveit_ros_move_group',
            executable='move_group',
            output='screen',
            parameters=[
                {'robot_description': open(urdf_path).read()},
                {'robot_description_semantic': open(srdf_path).read()},
            ]
        ),       
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[
                {'robot_description': open(urdf_path).read()},
                {'robot_description_semantic': open(srdf_path).read()},
            ],
            on_exit=Shutdown()
        ),
        Node(
            package='cxx_bridge', 
            executable='protobuf_server', 
            name='cxx_bridge', 
            output='screen',
            arguments=["5555"],
            parameters=[
                {'port': '5555'}, 
                {'pose_topic': 'pose_array'}, 
                {'trajectory_topic': 'pose_array'},
                {'pose_scaling': '0.005'}
            ],
            on_exit=Shutdown()            
        ),        
    ])

