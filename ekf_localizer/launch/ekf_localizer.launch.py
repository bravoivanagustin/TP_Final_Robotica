from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Variables que referencian los argumentos declarados mas abajo
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        # Argumento: usar el reloj del simulador (CoppeliaSim publica /clock).
        # Con use_sim_time=true todos los nodos consumen tiempo simulado; esto
        # garantiza que los stamps de los mensajes sean consistentes con /clock.
        # NUNCA usar rclcpp::Clock::now() en los nodos bajo use_sim_time=true.
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Usar el reloj del simulador (CoppeliaSim publica /clock).'
        ),

        # Argumento: archivo de mapa (no utilizado directamente por el EKF).
        # El mapa real de postes llega por topico /posts (geometry_msgs/PoseArray,
        # QoS transient_local) publicado por CoppeliaSim; no se carga desde YAML.
        # Se declara para compatibilidad con launch includes que lo pasen.
        DeclareLaunchArgument(
            'map_file',
            default_value='',
            description='Ruta a un archivo de mapa (no usado; el mapa llega por /posts).'
        ),

        # Nodo detector de landmarks (Etapa A — prerrequisito del EKF).
        # Convierte el scan LiDAR en observaciones (range, bearing) en frame
        # base_link y las publica en /landmarks (robmovil_msgs/LandmarkArray).
        # El EKF no corre la correccion si este nodo no esta activo.
        Node(
            package='ekf_localizer',
            executable='landmark_detector_node',
            name='landmark_detector',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
        ),

        # Nodo EKF de localizacion 2D (Etapa C).
        # Estado x=[px, py, theta] en frame map; usa Eigen puro (sin kfilter).
        # Prediccion: delta de odometria en body desde /robot/odometry.
        # Correccion: landmarks de /landmarks asociados contra el mapa de /posts.
        # Publica TF map -> base_link_ekf y /robot/odometry_ekf con covarianza.
        Node(
            package='ekf_localizer',
            executable='ekf_localizer',
            name='ekf_localizer',
            output='screen',
            parameters=[{
                'use_sim_time':         use_sim_time,

                # --- Frames ---
                # frame_id del mapa global (origen del estado del EKF)
                'map_frame':            'map',
                # child_frame_id publicado en la TF y en /robot/odometry_ekf
                'ekf_frame':            'base_link_ekf',

                # --- Ruido de proceso Q = diag(sigma_dx^2, sigma_dy^2, sigma_dth^2) ---
                # u=[dxb,dyb,dth] son deltas de distancia (m/paso), NO velocidades;
                # Q_base_ se usa directamente sin escalar por dt.
                'sigma_dx':             0.15,
                'sigma_dy':             0.15,
                'sigma_dth':            0.15,

                # --- Ruido de medicion R = diag(sigma_range^2, sigma_bearing^2) ---
                'sigma_range':          0.05,
                'sigma_bearing':        0.03,

                # --- Covarianza inicial P0 = I * init_cov ---
                # Valor pequeno: el estado se inicializa desde TF map->base_link,
                # que ya es bastante preciso.
                'init_cov':             0.01,

                # --- Data association: nearest-neighbor con gating ---
                # Umbral de distancia euclidea en frame map (metros).
                # Observaciones cuya proyeccion diste mas de este valor del poste
                # mas cercano se descartan como falsos positivos o ambiguedades.
                'association_max_dist': 0.6,

                # --- Distancia minima de un landmark para ser procesado (metros) ---
                # Landmarks muy cercanos son numericamente inestables (q->0 en Jh).
                'min_range':            0.2,
            }],
        ),
    ])
