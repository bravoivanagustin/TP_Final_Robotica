from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Seguimiento del cuadrado con feedback por odometria pura (sin EKF).
    #
    # Levanta el combo minimo de modulo 1 + modulo 2 en un solo proceso para
    # evitar la coreografia de dos terminales ('ros2 launch modelo_holonomico ...'
    # en una + 'ros2 launch lazo_cerrado ...' en otra).
    #
    # Paralelo a robmovil_ekf/launch/seguimiento_ekf.launch.py, que hace lo
    # mismo pero cerrando el lazo sobre base_link_ekf en vez de base_link.
    #
    # Args expuestos arriba (default sensato para que el launcher sin overrides
    # haga lo mismo que corriamos antes en dos terminales):
    #   - use_sim_time : reloj del sim.
    #   - kp_xy, kp_yaw: ganancias del controlador P (barridos del informe).
    #   - goal_mode    : PURSUIT / FIXED (switch para sanity checks).
    #
    # El resto de los args de lazo_cerrado.launch.py (lookahead_distance,
    # position_tolerance, square_half_side, etc.) heredan sus defaults internos.
    # Si en algun momento haya que barrerlos tambien, agregar aca en una linea.

    use_sim_time = LaunchConfiguration('use_sim_time')
    kp_xy        = LaunchConfiguration('kp_xy')
    kp_yaw       = LaunchConfiguration('kp_yaw')
    goal_mode    = LaunchConfiguration('goal_mode')

    return LaunchDescription([

        # ------------------------------------------------------------------ #
        # Argumentos tuneables desde CLI (patron del proyecto: se declaran
        # arriba, se referencian como LaunchConfiguration mas abajo).
        # ------------------------------------------------------------------ #
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Usar el reloj de CoppeliaSim (/clock).'
        ),
        DeclareLaunchArgument(
            'kp_xy',
            default_value='0.8',
            description='Ganancia P de traslacion del controlador (frame body).'
        ),
        DeclareLaunchArgument(
            'kp_yaw',
            default_value='0.75',
            description='Ganancia P de orientacion del controlador.'
        ),
        DeclareLaunchArgument(
            'goal_mode',
            default_value='PURSUIT',
            description='PURSUIT (seguimiento del cuadrado) o FIXED (sanity check de un goal).'
        ),

        # ------------------------------------------------------------------ #
        # Modulo 1: odometria.
        # Lanza mecanum_odometry_node, que publica:
        #   - /robot/odometry (nav_msgs/Odometry, frame odom)
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
        # Modulo 2: controlador a lazo cerrado + generador del cuadrado.
        # Feedback por TF map->base_link (pose por odometria pura, sin EKF).
        # Nota: map->odom la publica CoppeliaSim desde la escena, asi que la
        # cadena map->odom->base_link queda completa sin necesidad de agregar
        # una TF estatica (a diferencia de seguimiento_ekf.launch.py, que si
        # la necesita porque el EKF se inicializa desde ese lookup).
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
                'kp_xy':        kp_xy,
                'kp_yaw':       kp_yaw,
                'goal_mode':    goal_mode,
                'base_frame':   'base_link',
            }.items(),
        ),

    ])
