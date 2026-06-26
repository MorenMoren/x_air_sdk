from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1. 声明外部可配置的参数（定义默认的话题名称）
    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/cam_chest/cam_chest/depth/image_rect_raw',
        description='Topic name for input depth image'
    )
    
    camera_info_topic_arg = DeclareLaunchArgument(
        'camera_info_topic',
        default_value='/cam_chest/cam_chest/depth/camera_info',
        description='Topic name for input camera info'
    )
    
    point_cloud_topic_arg = DeclareLaunchArgument(
        'point_cloud_topic',
        default_value='/camera/depth/points',
        description='Topic name for output point cloud'
    )

    # 2. 创建一个组件容器 (Component Container)，相当于 ROS 1 的 Nodelet Manager
    container = ComposableNodeContainer(
        name='depth_image_proc_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # 3. 在容器中加载 depth_image_proc::PointCloudXyzNode 组件
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzNode',
                name='point_cloud_xyz_node',
                # 4. 关键：进行话题重映射 (Remap)
                remappings=[
                    ('image_rect', LaunchConfiguration('depth_topic')),
                    ('camera_info', LaunchConfiguration('camera_info_topic')),
                    ('points', LaunchConfiguration('point_cloud_topic'))
                ]
            )
        ],
        output='screen',
    )

    # 5. 将参数和容器返回给 Launch 系统
    return LaunchDescription([
        depth_topic_arg,
        camera_info_topic_arg,
        point_cloud_topic_arg,
        container
    ])
