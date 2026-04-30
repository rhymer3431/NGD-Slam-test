from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_no", default_value=""),
            DeclareLaunchArgument("width", default_value="640"),
            DeclareLaunchArgument("height", default_value="480"),
            DeclareLaunchArgument("fps", default_value="30"),
            DeclareLaunchArgument("color_topic", default_value="/camera/color/image_raw"),
            DeclareLaunchArgument(
                "depth_topic", default_value="/camera/aligned_depth_to_color/image_raw"
            ),
            DeclareLaunchArgument(
                "color_info_topic", default_value="/camera/color/camera_info"
            ),
            DeclareLaunchArgument(
                "depth_info_topic",
                default_value="/camera/aligned_depth_to_color/camera_info",
            ),
            DeclareLaunchArgument(
                "color_frame_id", default_value="camera_color_optical_frame"
            ),
            DeclareLaunchArgument(
                "depth_frame_id", default_value="camera_color_optical_frame"
            ),
            DeclareLaunchArgument("enable_auto_exposure", default_value="true"),
            DeclareLaunchArgument("emitter_enabled", default_value="true"),
            DeclareLaunchArgument("align_depth", default_value="true"),
            DeclareLaunchArgument("publish_camera_info", default_value="true"),
            Node(
                package="ngd_slam_ros",
                executable="realsense_d455_rgbd_node",
                name="realsense_d455_rgbd",
                output="screen",
                parameters=[
                    {
                        "serial_no": LaunchConfiguration("serial_no"),
                        "width": LaunchConfiguration("width"),
                        "height": LaunchConfiguration("height"),
                        "fps": LaunchConfiguration("fps"),
                        "color_topic": LaunchConfiguration("color_topic"),
                        "depth_topic": LaunchConfiguration("depth_topic"),
                        "color_info_topic": LaunchConfiguration("color_info_topic"),
                        "depth_info_topic": LaunchConfiguration("depth_info_topic"),
                        "color_frame_id": LaunchConfiguration("color_frame_id"),
                        "depth_frame_id": LaunchConfiguration("depth_frame_id"),
                        "enable_auto_exposure": LaunchConfiguration(
                            "enable_auto_exposure"
                        ),
                        "emitter_enabled": LaunchConfiguration("emitter_enabled"),
                        "align_depth": LaunchConfiguration("align_depth"),
                        "publish_camera_info": LaunchConfiguration(
                            "publish_camera_info"
                        ),
                    }
                ],
            ),
        ]
    )
