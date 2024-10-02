import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
import time
from ament_index_python.packages import get_package_share_directory

default_params = os.path.join(
    get_package_share_directory("picamera_ros2"), "config", "params.yaml"
)
mounted_params = "/config/camera/params.yaml"

params = mounted_params if os.path.exists(mounted_params) else default_params

current_time = time.strftime("%Y%m%d_%H%M", time.localtime(time.time() + 3 * 60 * 60))
print(f"current_time: {current_time}")


def generate_launch_description():
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
                parameters=[params],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            # Rosbag2 recorder
            ComposableNode(
                package="picamera_ros2",
                plugin="picamera_ros2::Recorder",
                name="recorder",
                parameters=[params],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    return launch.LaunchDescription([container])
