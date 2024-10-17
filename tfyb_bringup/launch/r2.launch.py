from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, LifecycleNode, Node
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution,
)
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import (
    get_package_share_directory,
    get_package_share_path,
)

import os


def generate_launch_description():

    # Arguments for Orbbec camera node
    color_fps_arg = DeclareLaunchArgument("color_fps", default_value="30")
    color_width_arg = DeclareLaunchArgument("color_width", default_value="640")
    color_height_arg = DeclareLaunchArgument("color_height", default_value="480")
    enable_depth_arg = DeclareLaunchArgument("enable_depth", default_value="false")
    enable_ir_arg = DeclareLaunchArgument("enable_ir", default_value="false")

    package_share_directory = FindPackageShare("yahboom_r2").find("tfyb_bringup")

    lidar_parameter_file = os.path.join(
        get_package_share_directory("ydlidar_ros2_driver"), "params", "ydlidar.yaml"
    )

    model_path = PathJoinSubstitution(
        [
            TextSubstitution(text=str(get_package_share_path("tfyb_bringup"))),
            "urdf",
            "yahboomcar_R2.urdf.xacro",
        ]
    )

    lidar_node = LifecycleNode(
        package="ydlidar_ros2_driver",
        executable="ydlidar_ros2_driver_node",
        name="ydlidar_ros2_driver_node",
        output="screen",
        emulate_tty=True,
        parameters=[lidar_parameter_file, {"frame_id": "laser_link"}],
        node_namespace="/",
    )

    # R2 Bringup
    yahboomcar_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                FindPackageShare("yahboomcar_bringup"),
                "/launch/yahboomcar_bringup_R2_launch.py",
            ]
        ),
        launch_arguments={"model": model_path}.items(),
    )

    # Start the joy_node
    joy_node = Node(
        package="joy", executable="joy_node", name="joy_node", output="screen"
    )

    # Container for composed image nodes
    container = ComposableNodeContainer(
        name="camera_processing_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",  # Multi-threaded container
        composable_node_descriptions=[
            # AprilTag node
            ComposableNode(
                package="apriltag_ros",
                plugin="AprilTagNode",
                name="apriltag_node",
                remappings=[("/camera_info", "/camera/color/camera_info")],
                parameters=[{package_share_directory + "/cfg/cn_tags_36h11.yaml"}],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            # Image Proc node
            ComposableNode(
                package="image_proc",
                plugin="image_proc::RectifyNode",
                name="rectify",
                remappings=[
                    ("image", "/camera/color/image_raw"),
                    ("camera_info", "/camera/color/camera_info"),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            # Orbbec camera driver node
            ComposableNode(
                package="orbbec_camera",
                plugin="orbbec_camera::OBCameraNodeDriver",
                name="camera",
                namespace="camera",
                parameters=[
                    {
                        "camera_name": "camera",
                        "color_fps": LaunchConfiguration("color_fps"),
                        "color_width": LaunchConfiguration("color_width"),
                        "color_height": LaunchConfiguration("color_height"),
                        "enable_depth": LaunchConfiguration("enable_depth"),
                        "enable_ir": LaunchConfiguration("enable_ir"),
                        "enable_color": True,
                    }
                ],
                #                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            color_fps_arg,
            color_width_arg,
            color_height_arg,
            enable_depth_arg,
            enable_ir_arg,
            lidar_node,
            yahboomcar_bringup,
            joy_node,
            container,
        ]
    )
