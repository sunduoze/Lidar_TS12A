from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 获取 delta_lidar 包的路径
    delta_2b_lidar_package_dir = get_package_share_directory('delta_2b_lidar')
    
    # 包含 delta_lidar 的 launch 文件
    delta_2b_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(delta_2b_lidar_package_dir, 'launch', 'delta_2b_lidar.launch.py')
        )
    )
    
    # 启动 RViz2 节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(delta_2b_lidar_package_dir, 'rviz', 'delta_2b_lidar.rviz')],
        output='screen'
    )
    
    return LaunchDescription([
        delta_2b_lidar_launch,
        rviz_node
    ])
