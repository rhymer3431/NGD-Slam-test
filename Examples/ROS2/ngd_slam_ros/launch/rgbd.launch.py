from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("ngd_slam_ros")
    default_settings = PathJoinSubstitution([pkg_share, "config", "RealSense_D455.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("voc_file", default_value=""),
            DeclareLaunchArgument("settings_file", default_value=default_settings),
            DeclareLaunchArgument("rgb_topic", default_value="/camera/color/image_raw"),
            DeclareLaunchArgument(
                "depth_topic", default_value="/camera/aligned_depth_to_color/image_raw"
            ),
            DeclareLaunchArgument("pose_topic", default_value="/ngd_slam/camera_pose"),
            DeclareLaunchArgument(
                "tracking_state_topic", default_value="/ngd_slam/tracking_state"
            ),
            DeclareLaunchArgument("map_frame", default_value="map"),
            DeclareLaunchArgument("camera_frame", default_value="camera"),
            DeclareLaunchArgument("queue_size", default_value="10"),
            DeclareLaunchArgument("use_viewer", default_value="true"),
            DeclareLaunchArgument("save_keyframe_trajectory", default_value=""),
            DeclareLaunchArgument("save_camera_trajectory", default_value=""),
            Node(
                package="ngd_slam_ros",
                executable="ngd_slam_rgbd_node",
                name="ngd_slam_rgbd",
                output="screen",
                parameters=[
                    {
                        "voc_file": LaunchConfiguration("voc_file"),
                        "settings_file": LaunchConfiguration("settings_file"),
                        "rgb_topic": LaunchConfiguration("rgb_topic"),
                        "depth_topic": LaunchConfiguration("depth_topic"),
                        "pose_topic": LaunchConfiguration("pose_topic"),
                        "tracking_state_topic": LaunchConfiguration("tracking_state_topic"),
                        "map_frame": LaunchConfiguration("map_frame"),
                        "camera_frame": LaunchConfiguration("camera_frame"),
                        "queue_size": LaunchConfiguration("queue_size"),
                        "use_viewer": LaunchConfiguration("use_viewer"),
                        "save_keyframe_trajectory": LaunchConfiguration(
                            "save_keyframe_trajectory"
                        ),
                        "save_camera_trajectory": LaunchConfiguration(
                            "save_camera_trajectory"
                        ),
                    }
                ],
            ),
        ]
    )
