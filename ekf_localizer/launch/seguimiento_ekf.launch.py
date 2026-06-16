from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Referencia al argumento global de reloj; se resuelve en tiempo de
    # lanzamiento para que sea sobreescribible desde la linea de comandos
    # (use_sim_time:=false para pruebas fuera del simulador).
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([

        # ------------------------------------------------------------------ #
        # Argumento global: reloj del simulador.
        # Todos los nodos de los tres modulos lo reciben via launch_arguments
        # para que los stamps de los mensajes sean consistentes con /clock.
        # ------------------------------------------------------------------ #
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Usar el reloj de CoppeliaSim (/clock).'
        ),

        # ------------------------------------------------------------------ #
        # TF estatica map->odom (identidad).
        #
        # mecanum_odometry publica odom->base_link pero nadie publica
        # map->odom. Sin esta TF el arbol queda partido en dos raices y el
        # EKF no puede hacer lookupTransform("map", "base_link") al
        # inicializarse (Etapa A).
        #
        # La identidad es valida porque la escena omni_ekf.ttt coloca al
        # robot exactamente en el origen del frame map en t=0.
        # ------------------------------------------------------------------ #
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # ------------------------------------------------------------------ #
        # Modulo 1: modelo holonomico.
        # Lanza mecanum_odometry_node, que publica:
        #   - /robot/odometry  (nav_msgs/Odometry, frame odom)
        #   - TF odom->base_link
        # ------------------------------------------------------------------ #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('modelo_holonomico'),
                    'launch',
                    'odometry.launch.py',
                ])
            ]),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # ------------------------------------------------------------------ #
        # Modulo 3: EKF localizer.
        # Lanza landmark_detector_node y ekf_localizer, que publican:
        #   - /robot/odometry_ekf  (nav_msgs/Odometry, frame map)
        #   - TF map->base_link_ekf
        # El EKF se inicializa desde lookupTransform("map", "base_link")
        # en el primer tick; la TF estatica de arriba garantiza que esa
        # cadena sea navegable desde el primer instante.
        # ------------------------------------------------------------------ #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ekf_localizer'),
                    'launch',
                    'ekf_localizer.launch.py',
                ])
            ]),
            launch_arguments={'use_sim_time': use_sim_time}.items(),
        ),

        # ------------------------------------------------------------------ #
        # Modulo 2: controlador a lazo cerrado.
        # Se pasa base_frame:=base_link_ekf para que KinematicHolonomicController
        # cierre el lazo sobre la pose estimada por el EKF en lugar de la
        # odometria pura. El controlador ya maneja TransformException y reintenta
        # en el siguiente tick si la TF aun no esta disponible.
        # ------------------------------------------------------------------ #
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('lazo_cerrado'),
                    'launch',
                    'lazo_cerrado.launch.py',
                ])
            ]),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map_frame':    'map',
                'base_frame':   'base_link_ekf',
            }.items(),
        ),

    ])
