from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    only_prediction = LaunchConfiguration('only_prediction')
    sigma_x = LaunchConfiguration('sigma_x')
    sigma_y = LaunchConfiguration('sigma_y')
    sigma_theta = LaunchConfiguration('sigma_theta')
    sigma_range = LaunchConfiguration('sigma_range')
    sigma_bearing = LaunchConfiguration('sigma_bearing')

    return LaunchDescription([
        DeclareLaunchArgument('only_prediction', default_value='false'),
        DeclareLaunchArgument('sigma_x', default_value='0.15'),
        DeclareLaunchArgument('sigma_y', default_value='0.15'),
        DeclareLaunchArgument('sigma_theta', default_value='0.15'),
        DeclareLaunchArgument('sigma_range', default_value='0.05'),
        DeclareLaunchArgument('sigma_bearing', default_value='0.03'),

        # Landmark detector (publica /landmarks en el frame base_link)
        Node(
            package="robmovil_ekf",
            executable="landmark_detector_node",
            name="landmark_detector",
            output="screen",
            parameters=[{"use_sim_time": True}]
        ),

        # EKF node
        Node(
            package="robmovil_ekf",
            executable="localizer",
            name="localizer",
            output="screen",
            parameters=[{"only_prediction": only_prediction},
                        {"sigma_x": sigma_x},
                        {"sigma_y": sigma_y},
                        {"sigma_theta": sigma_theta},
                        {"sigma_range": sigma_range},
                        {"sigma_bearing": sigma_bearing},
                        {"use_sim_time": True}]
        ),
    ])
