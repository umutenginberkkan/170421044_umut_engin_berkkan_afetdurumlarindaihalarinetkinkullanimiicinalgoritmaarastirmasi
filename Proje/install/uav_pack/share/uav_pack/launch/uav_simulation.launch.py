from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Komut satırından alacağımız argümanları tanımlıyoruz:
        DeclareLaunchArgument('num_uav',   default_value='5',  description='Number of UAVs'),
        DeclareLaunchArgument('formation', default_value='1',  description='Formation type (1,2,3,4,5)'),
        DeclareLaunchArgument('algorithm', default_value='1',  description='Algorithm (1=PSO,2=ACO,3=SMA,4=GWO,5=WOA)'),
        # uav_simulation.launch.py içinde:
        DeclareLaunchArgument("coverage_radius", default_value="10.0"),
        DeclareLaunchArgument("comm_threshold", default_value="20.0"),
        DeclareLaunchArgument("area_size", default_value="150.0"),
        DeclareLaunchArgument("dispersion_iters", default_value="1"),

        # UAV Simulator node’una parametre olarak veriyoruz:
        Node(
            package='uav_pack',
            executable='uav_node',
            name='uav_simulation',
            output='screen',
            parameters=[{
                'num_uav':   LaunchConfiguration('num_uav'),
                'formation': LaunchConfiguration('formation'),
                'algorithm': LaunchConfiguration('algorithm'),
                "coverage_radius": LaunchConfiguration("coverage_radius"),
                "comm_threshold": LaunchConfiguration("comm_threshold"),
                "area_size": LaunchConfiguration("area_size"),
                "dispersion_iters": LaunchConfiguration("dispersion_iters")
            }]
        )
    ])

