from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


# Igual que ekf_localizer/launch/seguimiento_ekf.launch.py, pero con el EKF
# implementado sobre EKFilter (robmovil_ekf) en lugar del de Eigen.
def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    def include(package, launch_file, arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare(package), 'launch', launch_file])
            ]),
            launch_arguments=arguments.items(),
        )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Usar el reloj de CoppeliaSim (/clock).'
        ),

        # TF estatica map->odom (identidad): el robot arranca en el origen del frame map.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # Modulo 1: /robot/odometry y TF odom->base_link
        include('modelo_holonomico', 'odometry.launch.py', {'use_sim_time': use_sim_time}),

        # Modulo 3: detector de postes + EKF (EKFilter)
        include('robmovil_ekf', 'localizer.launch.py', {'use_sim_time': use_sim_time}),

        # Modulo 2: el controlador cierra el lazo sobre la pose del EKF
        include('lazo_cerrado', 'lazo_cerrado.launch.py', {
            'use_sim_time': use_sim_time,
            'map_frame':    'map',
            'base_frame':   'base_link_ekf',
        }),
    ])
