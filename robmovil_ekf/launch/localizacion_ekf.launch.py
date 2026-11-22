from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

# Odometria + detector de postes + EKF, sin controlador. Es el backend de los
# experimentos de localizacion: el robot lo mueve otro driver (velocidades
# constantes o lazo_cerrado.launch.py con base_frame:=base_link_ekf).
# Los args de localizer.launch.py (sigma_x, only_prediction, ...) se pasan
# directo por linea de comandos.
# Lanzar con la simulacion detenida: /posts llega una sola vez al dar play.

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('modelo_holonomico'), 'launch', 'odometry.launch.py'))
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('robmovil_ekf'), 'launch', 'localizer.launch.py'))
        ),
    ])
