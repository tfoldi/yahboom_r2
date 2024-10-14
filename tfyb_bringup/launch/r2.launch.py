from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Arguments for Orbbec camera node
    color_fps_arg = DeclareLaunchArgument('color_fps', default_value='30')
    color_width_arg = DeclareLaunchArgument('color_width', default_value='640')
    color_height_arg = DeclareLaunchArgument('color_height', default_value='480')
    enable_depth_arg = DeclareLaunchArgument('enable_depth', default_value='false')
    enable_ir_arg = DeclareLaunchArgument('enable_ir', default_value='false')

    package_share_directory = FindPackageShare('yahboom_r2').find('tfyb_bringup')

    # Container for composed nodes
    container = ComposableNodeContainer(
        name='camera_processing_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # Multi-threaded container
        composable_node_descriptions=[
            # AprilTag node
            ComposableNode(
                package='apriltag_ros',
                plugin='AprilTagNode',
                name='apriltag_node',
                remappings=[('/camera_info', '/camera/color/camera_info')],
                parameters=[{package_share_directory + '/cfg/cn_tags_36h11.yaml'}],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # Image Proc node
            ComposableNode(
                package='image_proc',
                plugin='image_proc::RectifyNode',
                name='rectify',
                remappings=[
                    ('image', '/camera/color/image_raw'),
                    ('camera_info', '/camera/color/camera_info')
                ],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            # Orbbec camera driver node
            ComposableNode(
                package='orbbec_camera',
                plugin='orbbec_camera::OBCameraNodeDriver',
                name='camera',
                namespace='camera',
                parameters=[{
                    'color_fps': LaunchConfiguration('color_fps'),
                    'color_width': LaunchConfiguration('color_width'),
                    'color_height': LaunchConfiguration('color_height'),
                    'enable_depth': LaunchConfiguration('enable_depth'),
                    'enable_ir': LaunchConfiguration('enable_ir'),
                    'enable_color': True,
                }],
#                extra_arguments=[{'use_intra_process_comms': True}]
            ),
        ],
        output='screen',
    )

    return LaunchDescription([
        color_fps_arg,
        color_width_arg,
        color_height_arg,
        enable_depth_arg,
        enable_ir_arg,
        container
    ])

