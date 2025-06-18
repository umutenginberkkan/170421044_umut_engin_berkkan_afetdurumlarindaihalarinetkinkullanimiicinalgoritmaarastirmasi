from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Komut satırından alacağımız argümanları tanımlıyoruz:
        DeclareLaunchArgument('num_uav',   default_value='5',  description='Number of UAVs'),
        DeclareLaunchArgument('formation', default_value='1',  description='Formation type (1,2,3)'),
        DeclareLaunchArgument('algorithm', default_value='1',  description='Algorithm (1=PSO,2=ACO)'),

        # UAV Simulator node’una parametre olarak veriyoruz:
        Node(
            package='uav_pack',
            executable='uav_node',
            name='uav_simulation',
            output='screen',
            parameters=[{
                'num_uav':   LaunchConfiguration('num_uav'),
                'formation': LaunchConfiguration('formation'),
                'algorithm': LaunchConfiguration('algorithm')
            }]
        )
    ])

