import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
import time
from ament_index_python.packages import get_package_share_directory

params = os.path.join(
    get_package_share_directory("picamera_ros2"), "config", "params.yaml"
)


current_time = time.strftime("%Y%m%d_%H%M", time.localtime(time.time() + 3 * 60 * 60))


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            "bag_uri",
            default_value=f"/bags/camera_{current_time}",
            description="URI for the rosbag storage.",
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
                parameters=[[params]],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            # Rosbag2 recorder
            ComposableNode(
                package="rosbag2_transport",
                plugin="rosbag2_transport::Recorder",
                name="rosbag2_recorder",
                extra_arguments=[{"use_intra_process_comms": True}],
                parameters=[
                    params,
                    {
                        "storage": {
                            "uri": LaunchConfiguration("bag_uri"),
                        },
                    },
                ],
            ),
        ],
        output="screen",
    )

    return launch.LaunchDescription(launch_args + [container])
