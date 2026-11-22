from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([

        # map -> odom lo publica el simulador (pose de spawn), no se publica aca

        # Modelo cinematico: /robot/odometry y TF odom -> base_link
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('modelo_holonomico'), 'launch', 'odometry.launch.py'))
        ),

        # Detector de postes + EKF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('robmovil_ekf'), 'launch', 'localizer.launch.py'))
        ),

        # Control a lazo cerrado sobre la pose estimada por el EKF
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('lazo_cerrado'), 'launch', 'lazo_cerrado.launch.py')),
            launch_arguments={'map_frame': 'map',
                              'base_frame': 'base_link_ekf'}.items()
        ),
    ])
