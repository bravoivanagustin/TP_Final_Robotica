from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Variables que referencian los argumentos declarados mas abajo
    use_sim_time = LaunchConfiguration('use_sim_time')
    scan_topic   = LaunchConfiguration('scan_topic')

    return LaunchDescription([
        # Argumento: usar el reloj del simulador (CoppeliaSim publica /clock).
        # Con use_sim_time=true todos los nodos consumen tiempo simulado; esto
        # garantiza que los stamps de los mensajes sean consistentes con /clock.
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Usar el reloj del simulador (CoppeliaSim publica /clock).'
        ),

        # Argumento: topico del scan LiDAR frontal.
        # Permite redirigir el detector a un topico distinto sin recompilar,
        # por ejemplo durante pruebas con un bag o con otro robot.
        DeclareLaunchArgument(
            'scan_topic',
            default_value='/robot/front_laser/scan',
            description='Topico de entrada del scan LiDAR (sensor_msgs/LaserScan).'
        ),

        # Nodo detector de landmarks: convierte el scan LiDAR en una lista de
        # postes candidatos (range, bearing) en frame base_link y los publica
        # en /landmarks (robmovil_msgs/LandmarkArray) y en /landmark_markers
        # (visualization_msgs/MarkerArray) para visualizacion en RViz2.
        # El nodo es stateless respecto al mapa: no conoce /posts ni asigna IDs.
        Node(
            package='ekf_localizer',
            executable='landmark_detector_node',
            name='landmark_detector',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'scan_topic':   scan_topic,
            }],
        ),
    ])
