from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="pd_sim",
            executable="pd_sim_node",
            name="custom_minimal_param_node",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"robot_model": "/home/dadrozdov/repos/projective-dynamics-for-robotics/models/robot_simple.gltf"}
            ]
        )
    ])