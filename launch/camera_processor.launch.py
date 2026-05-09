"""
Launch the ida_camera processor node.

Usage examples:
  ros2 launch ida_camera camera_processor.launch.py
  ros2 launch ida_camera camera_processor.launch.py use_clahe:=false
  ros2 launch ida_camera camera_processor.launch.py use_sharpen:=true use_gamma:=true gamma:=0.7
  ros2 launch ida_camera camera_processor.launch.py \\
      yaml_calibration_path:=/path/to/c920_calibration.yaml undistort_alpha:=0.5

Runtime tuning (no restart needed):
  ros2 param set /camera_processor clahe_clip_limit 3.5
  ros2 param set /camera_processor use_denoise true
  ros2 param set /camera_processor gamma 0.8
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("ida_camera")

    args = [
        # Calibration
        DeclareLaunchArgument("yaml_calibration_path", default_value="",
            description="Absolute path to camera calibration YAML."),
        DeclareLaunchArgument("undistort_alpha", default_value="0.0",
            description="0=tight crop, 1=full frame with black borders."),
        # Denoise
        DeclareLaunchArgument("use_denoise", default_value="false",
            description="Enable bilateral denoising (runs before CLAHE)."),
        # CLAHE
        DeclareLaunchArgument("use_clahe", default_value="true",
            description="Enable CLAHE contrast enhancement."),
        DeclareLaunchArgument("clahe_clip_limit", default_value="1.5",
            description="CLAHE contrast cap per tile."),
        # Sharpen
        DeclareLaunchArgument("use_sharpen", default_value="false",
            description="Enable unsharp mask sharpening."),
        DeclareLaunchArgument("sharpen_strength", default_value="0.8",
            description="Sharpening blend weight (0=none, 1=full)."),
        # Gamma
        DeclareLaunchArgument("use_gamma", default_value="false",
            description="Enable gamma correction."),
        DeclareLaunchArgument("gamma", default_value="1.0",
            description="Gamma exponent. <1=brighter, >1=darker, 2.2=linearise sRGB."),
        # ML output
        DeclareLaunchArgument("ml_topic", default_value="/camera/image_ml",
            description="Topic for the letterboxed RGB image consumed by the YOLO node."),
        DeclareLaunchArgument("ml_width",  default_value="640",
            description="ML output image width."),
        DeclareLaunchArgument("ml_height", default_value="640",
            description="ML output image height."),
    ]

    node = Node(
        package="ida_camera",
        executable="camera_processor_node",
        name="camera_processor",
        output="screen",
        parameters=[
            PathJoinSubstitution([pkg_share, "config", "camera_params.yaml"]),
            {
                "yaml_calibration_path": LaunchConfiguration("yaml_calibration_path"),
                "undistort_alpha":       LaunchConfiguration("undistort_alpha"),
                "use_denoise":           LaunchConfiguration("use_denoise"),
                "use_clahe":             LaunchConfiguration("use_clahe"),
                "clahe_clip_limit":      LaunchConfiguration("clahe_clip_limit"),
                "use_sharpen":           LaunchConfiguration("use_sharpen"),
                "sharpen_strength":      LaunchConfiguration("sharpen_strength"),
                "use_gamma":             LaunchConfiguration("use_gamma"),
                "gamma":                 LaunchConfiguration("gamma"),
                "ml_topic":              LaunchConfiguration("ml_topic"),
                "ml_width":              LaunchConfiguration("ml_width"),
                "ml_height":             LaunchConfiguration("ml_height"),
            },
        ],
    )

    return LaunchDescription(args + [node])
