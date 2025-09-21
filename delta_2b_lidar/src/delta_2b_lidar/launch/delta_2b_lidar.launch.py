from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明启动参数
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for the Delta 2b LiDAR'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='laser',
        description='Frame ID for the LiDAR data'
    )
    
    # 创建 Delta LiDAR 节点
    delta_2b_lidar_node = Node(
        package='delta_2b_lidar',
        executable='delta_2b_lidar_node',
        name='delta_2b_lidar',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'frame_id': LaunchConfiguration('frame_id')
        }]
    )
    
    return LaunchDescription([
        serial_port_arg,
        frame_id_arg,
        delta_2b_lidar_node
    ])
