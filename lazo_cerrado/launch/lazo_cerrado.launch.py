from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Reloj.
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Trayectoria.
    square_half_side = LaunchConfiguration('square_half_side')
    waypoint_step    = LaunchConfiguration('waypoint_step')
    frame_id         = LaunchConfiguration('frame_id')

    # Modo del controlador.
    goal_mode       = LaunchConfiguration('goal_mode')
    fixed_goal_x    = LaunchConfiguration('fixed_goal_x')
    fixed_goal_y    = LaunchConfiguration('fixed_goal_y')
    fixed_goal_yaw  = LaunchConfiguration('fixed_goal_yaw')

    # Ganancias y tolerancias.
    kp_xy              = LaunchConfiguration('kp_xy')
    kp_yaw             = LaunchConfiguration('kp_yaw')
    lookahead_distance = LaunchConfiguration('lookahead_distance')
    position_tolerance = LaunchConfiguration('position_tolerance')
    yaw_tolerance      = LaunchConfiguration('yaw_tolerance')
    control_rate_hz    = LaunchConfiguration('control_rate_hz')

    # Frames del feedback (cambiables a base_link_ekf cuando entre módulo 3).
    map_frame  = LaunchConfiguration('map_frame')
    base_frame = LaunchConfiguration('base_frame')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Usar el reloj del simulador (CoppeliaSim publica /clock).'),

        # Trayectoria.
        DeclareLaunchArgument('square_half_side', default_value='2.0',
                              description='Semi-lado del cuadrado en metros (vértices ±s, ±s).'),
        DeclareLaunchArgument('waypoint_step', default_value='0.05',
                              description='Paso de discretización de la trayectoria en metros.'),
        DeclareLaunchArgument('frame_id', default_value='map',
                              description='Frame en el header de la Trajectory y del Path.'),

        # Modo de selección de goal.
        DeclareLaunchArgument('goal_mode', default_value='PURSUIT',
                              description='FIXED (sanity checks) o PURSUIT (cuadrado).'),
        DeclareLaunchArgument('fixed_goal_x', default_value='2.0',
                              description='Goal X en modo FIXED.'),
        DeclareLaunchArgument('fixed_goal_y', default_value='2.0',
                              description='Goal Y en modo FIXED.'),
        DeclareLaunchArgument('fixed_goal_yaw', default_value='0.785398163',
                              description='Goal yaw en modo FIXED (rad). Default π/4.'),

        # Ganancias, saturaciones, tolerancias.
        DeclareLaunchArgument('kp_xy', default_value='0.8',
                              description='Ganancia P de traslación (frame body).'),
        DeclareLaunchArgument('kp_yaw', default_value='1.5',
                              description='Ganancia P de orientación.'),
        DeclareLaunchArgument('lookahead_distance', default_value='0.30',
                              description='Distancia [m] mínima del waypoint perseguido al robot.'),
        DeclareLaunchArgument('position_tolerance', default_value='0.05',
                              description='Tolerancia [m] para considerar fin de trayectoria.'),
        DeclareLaunchArgument('yaw_tolerance', default_value='0.05',
                              description='Tolerancia angular [rad] para considerar fin de trayectoria.'),
        DeclareLaunchArgument('control_rate_hz', default_value='30.0',
                              description='Frecuencia del timer de control en Hz.'),

        # Frames.
        DeclareLaunchArgument('map_frame', default_value='map',
                              description='Frame del setpoint y del feedback.'),
        DeclareLaunchArgument('base_frame', default_value='base_link',
                              description='Frame del robot (cambiar a base_link_ekf en módulo 3).'),

        # Generador de la trayectoria cuadrada.
        Node(
            package='lazo_cerrado',
            executable='trajectory_generator',
            name='trajectory_generator',
            output='screen',
            parameters=[{
                'use_sim_time':      use_sim_time,
                'square_half_side':  square_half_side,
                'waypoint_step':     waypoint_step,
                'frame_id':          frame_id,
            }],
        ),

        # Controlador a lazo cerrado.
        Node(
            package='lazo_cerrado',
            executable='trajectory_follower',
            name='trajectory_follower',
            output='screen',
            parameters=[{
                'use_sim_time':        use_sim_time,
                'goal_mode':           goal_mode,
                'fixed_goal_x':        fixed_goal_x,
                'fixed_goal_y':        fixed_goal_y,
                'fixed_goal_yaw':      fixed_goal_yaw,
                'kp_xy':               kp_xy,
                'kp_yaw':              kp_yaw,
                'lookahead_distance':  lookahead_distance,
                'position_tolerance':  position_tolerance,
                'yaw_tolerance':       yaw_tolerance,
                'control_rate_hz':     control_rate_hz,
                'map_frame':           map_frame,
                'base_frame':          base_frame,
            }],
        ),
    ])
