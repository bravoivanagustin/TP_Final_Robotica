from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Parametros del filtro que se pueden barrer desde la linea de comandos.
# Defaults iguales a los de ekf_localizer, para poder comparar ambas implementaciones.
EKF_ARGS = [
    ('sigma_x',            '0.15', 'Desvio del ruido de proceso en x por paso de prediccion [m].'),
    ('sigma_y',            '0.15', 'Desvio del ruido de proceso en y por paso de prediccion [m].'),
    ('sigma_theta',        '0.15', 'Desvio del ruido de proceso en theta por paso de prediccion [rad].'),
    ('sigma_range',        '0.05', 'Desvio del ruido de medicion en distancia [m].'),
    ('sigma_bearing',      '0.03', 'Desvio del ruido de medicion en angulo [rad].'),
    ('init_cov',           '0.01', 'Covarianza inicial P0 = init_cov * I.'),
    ('association_radius', '0.6',  'Radio de la asociacion nearest-neighbor en el frame map [m].'),
    ('min_range',          '0.2',  'Distancia minima de un landmark para usarlo en la correccion [m].'),
    ('only_prediction',    'false', 'Desactivar la correccion (solo odometria propagada por el filtro).'),
]


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    args = [DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Usar el reloj del simulador (CoppeliaSim publica /clock).'
    )]
    args += [DeclareLaunchArgument(name, default_value=default, description=desc)
             for name, default, desc in EKF_ARGS]

    ekf_params = {name: LaunchConfiguration(name) for name, _, _ in EKF_ARGS}
    ekf_params.update({
        'use_sim_time':    use_sim_time,
        'ekf_frame':       'map',
        'base_link_frame': 'base_link_ekf',
        'odom_base_frame': 'base_link',
    })

    return LaunchDescription(args + [
        # El detector de postes es el mismo que usa ekf_localizer:
        # publica /landmarks (range, bearing) en el frame base_link.
        Node(
            package='ekf_localizer',
            executable='landmark_detector_node',
            name='landmark_detector',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # EKF sobre la libreria EKFilter (kfilter).
        # Publica TF map -> base_link_ekf y /robot/odometry_ekf con covarianza.
        Node(
            package='robmovil_ekf',
            executable='localizer',
            name='localizer',
            output='screen',
            parameters=[ekf_params],
        ),
    ])
