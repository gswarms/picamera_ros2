import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import time


def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            "video_width",
            default_value="640",
            description="Width of the video stream."
        ),
        DeclareLaunchArgument(
            "video_height",
            default_value="480",
            description="Height of the video stream."
        ),
        DeclareLaunchArgument(
            "framerate",
            default_value="30",
            description="Framerate of the video stream."
        ),
        DeclareLaunchArgument(
            "shutter",
            default_value="1000.0",
            description="Shutter speed of the camera in milliseconds."
        ),
        DeclareLaunchArgument(
            "storage_id",
            default_value="mcap",
            description="Storage format for the bag file (sqlite3, mcap, etc.)"
        ),
        DeclareLaunchArgument(
            "bag_output_directory",
            default_value="/path/to/rosbags",
            description="Path to store the rosbag."
        ),
        DeclareLaunchArgument(
            "camera_topic",
            default_value="/camera/image_raw",
            description="Topic to record."
        ),
    ]

    # current_time = time.strftime('%Y-%m-%d-%H-%M-%S', time.localtime())

    container = ComposableNodeContainer(
        name='picamera_ros2_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # Camera node
            ComposableNode(
                package='picamera_ros2',
                plugin='picamera_ros::PiCameraROS',
                name='picamera_ros2',
                parameters=[{
                    'video_width': LaunchConfiguration('video_width'),
                    'video_height': LaunchConfiguration('video_height'),
                    'framerate': LaunchConfiguration('framerate'),
                }]
            ),
            # Rosbag2 recorder node to record camera topic only
            # ComposableNode(
            #     package='rosbag2_transport',
            #     plugin='rosbag2_transport::Recorder',
            #     name='rosbag2_recorder',
            #     parameters=[{
            #         'ros__parameters': {
            #             'record': {
            #                 'topics': [LaunchConfiguration('camera_topic')],
            #                 'storage_id': LaunchConfiguration('storage_id'),
            #                 'uri': f"/bags/{current_time}",
            #             },
            #         }
            #     }]
            # ),
        ],
        output='screen',
    )

    return launch.LaunchDescription(
        launch_args +
        [
            container
        ]
    )
