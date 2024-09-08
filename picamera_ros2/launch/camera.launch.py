import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory

params = os.path.join(
    get_package_share_directory("picamera_ros2"), "config", "params.yaml"
)


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            "video_width", default_value="640", description="Width of the video stream."
        ),
        DeclareLaunchArgument(
            "video_height",
            default_value="480",
            description="Height of the video stream.",
        ),
        DeclareLaunchArgument(
            "framerate",
            default_value="30",
            description="Framerate of the video stream.",
        ),
        DeclareLaunchArgument(
            "shutter",
            default_value="1000.0",
            description="Shutter speed of the camera in milliseconds.",
        ),
    ]

    container = ComposableNodeContainer(
        name="picamera_ros2_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            # Camera node
            ComposableNode(
                package="picamera_ros2",
                plugin="picamera_ros::PiCameraROS",
                name="picamera_ros2",
                parameters=[
                    {
                        "video_width": LaunchConfiguration("video_width"),
                        "video_height": LaunchConfiguration("video_height"),
                        "framerate": LaunchConfiguration("framerate"),
                        "shutter": LaunchConfiguration("shutter"),
                    }
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            # Rosbag2 recorder node to record camera topic only
            ComposableNode(
                package="rosbag2_transport",
                plugin="rosbag2_transport::Recorder",
                name="rosbag2_recorder",
                extra_arguments=[{"use_intra_process_comms": True}],
                parameters=[params],
            ),
        ],
        output="screen",
    )

    return launch.LaunchDescription(launch_args + [container])
