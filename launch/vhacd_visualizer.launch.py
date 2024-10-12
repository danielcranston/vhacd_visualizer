import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    os.chdir("/tmp")  # Or is there a way to set the cwd for a Node like in ROS1?

    # Declare arguments
    declared_arguments = [
        DeclareLaunchArgument("run_rviz", default_value="true", description="Whether to run RViz"),
        DeclareLaunchArgument("mesh_filename", description="Full path to the input mesh"),
        DeclareLaunchArgument(
            "vhacd_executable_path",
            default_value="TestVHACD",
            description="Path to the 'TestVHACD' executable",
        ),
    ]

    # Initialize Arguments
    run_rviz = LaunchConfiguration("run_rviz")
    vhacd_executable_path = LaunchConfiguration("vhacd_executable_path")
    mesh_filename = LaunchConfiguration("mesh_filename")

    vhacd_visualizer_node = Node(
        package="vhacd_visualizer",
        executable="vhacd_visualizer",
        name="vhacd_visualizer",
        output="log",
        parameters=[
            {"mesh_filename": mesh_filename, "vhacd_executable_path": vhacd_executable_path}
        ],
    )

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("vhacd_visualizer"), "config/vhacd_visualizer.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(run_rviz),
    )

    rqt_reconfigure_node = Node(
        package="rqt_reconfigure",
        executable="rqt_reconfigure",
        name="rqt_reconfigure",
        output="log",
    )

    nodes_to_start = [vhacd_visualizer_node, rviz_node, rqt_reconfigure_node]

    return LaunchDescription(declared_arguments + nodes_to_start)
