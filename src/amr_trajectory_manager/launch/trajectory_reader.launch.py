from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "trajectory_file",
            default_value="/home/amith/work/amr_trajectory/trajectory1.csv",
            description="Path to the trajectory file"
        ),
        DeclareLaunchArgument(
            "trajectory_topic",
            default_value="/custom_trajectory_data",
            description="Topic for trajectory data"
        ),
        DeclareLaunchArgument(
            "marker_topic",
            default_value="/custom_trajectory_markers",
            description="Topic for trajectory markers"
        ),

        Node(
            package="amr_trajectory_manager",
            executable="trajectory_follower",
            name="trajectory_reader",
            parameters=[{
                "trajectory_file": LaunchConfiguration("trajectory_file"),
                "trajectory_topic": LaunchConfiguration("trajectory_topic"),
                "marker_topic": LaunchConfiguration("marker_topic")
            }]
        )
    ])
