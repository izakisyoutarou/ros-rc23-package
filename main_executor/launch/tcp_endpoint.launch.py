from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="ros_tcp_endpoint",
                executable="default_server_endpoint",
                name='ros_tcp_endpoint_1',
                namespace='ros_tcp_endpoint_1',
                emulate_tty=True,
                parameters=[{"ROS_IP": "0.0.0.0"}, {"ROS_TCP_PORT": 10000}],
            ),
            Node(
                package="ros_tcp_endpoint",
                executable="default_server_endpoint",
                name='ros_tcp_endpoint_2',
                namespace='ros_tcp_endpoint_2',
                emulate_tty=True,
                parameters=[{"ROS_IP": "0.0.0.0"}, {"ROS_TCP_PORT": 11000}],
            )
        ]
    )
