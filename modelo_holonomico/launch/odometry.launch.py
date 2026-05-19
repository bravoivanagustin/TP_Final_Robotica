from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Usar el reloj del simulador (CoppeliaSim publica /clock).'
        ),
        Node(
            package='modelo_holonomico',
            executable='mecanum_odometry_node',
            name='mecanum_odometry',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
