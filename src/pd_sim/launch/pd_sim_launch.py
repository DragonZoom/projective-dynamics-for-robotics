from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_model',
            default_value='models/robot_simple.gltf',
            description='Path to the robot model'
        ),
        Node(
            package="pd_sim",
            executable="pd_sim_node",
            name="custom_minimal_param_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"robot_model": LaunchConfiguration('robot_model')}
            ]
        )
    ])