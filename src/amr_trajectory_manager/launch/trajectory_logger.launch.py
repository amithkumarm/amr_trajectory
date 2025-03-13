from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments with default values
        DeclareLaunchArgument("odom_topic", default_value="/odom", description="Odometry topic to subscribe to"),
        DeclareLaunchArgument("marker_topic", default_value="/trajectory_markers", description="Marker topic for visualization"),
        DeclareLaunchArgument("trajectory_directory", default_value="/home/amith/", description="Directory to store trajectory files"),
        
        # Node declaration using the launch arguments
        Node(
            package="amr_trajectory_manager",
            executable="trajectory_publisher_saver",
            name="trajectory_logger",
            output="screen",
            parameters=[{
                "odom_topic": LaunchConfiguration("odom_topic"),
                "marker_topic": LaunchConfiguration("marker_topic"),
                "trajectory_directory": LaunchConfiguration("trajectory_directory")
            }]
        )
    ])
