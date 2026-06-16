# Etapa D — Integración con Módulo 2: Seguimiento con pose EKF (ítem 5 de la consigna)

---

## Prerrequisitos

- **Etapa A** completada: nodo `ekf_localizer` compila, suscribe a `/posts` (QoS transient\_local), inicializa el estado desde la primera TF `map→base_link`, y publica `/robot/odometry_ekf` (nav\_msgs/Odometry).
- **Etapa B** completada: predicción EKF con control input `u = [dxb, dyb, dth]` (deltas de odometría en frame body, igual que `mecanum_odometry.cpp:103-110`), sin errores de integración en trayectorias rectilíneas y circulares.
- **Etapa C** completada: actualización EKF con medición de bearing a postes; residuo normalizado con `angles::normalize_angle`; la elipse de covarianza converge visiblemente en RViz cuando hay postes en el FoV del laser.
- La TF `map→base_link_ekf` se publica correctamente desde `ekf_localizer` (verificado con `ros2 run tf2_tools view_frames`).

---

## Paso 1 — Verificar contrato de frames del controlador

**Objetivo:** confirmar que `KinematicHolonomicController` puede usar `base_link_ekf` como frame del robot sin modificar ningún archivo de código fuente; el cambio se hace exclusivamente por parámetro.

**Lectura relevante de `KinematicHolonomicController.cpp`:**

```cpp
// Parámetros configurables en tiempo de lanzamiento:
map_frame_  = this->declare_parameter("map_frame",  std::string("map"));
base_frame_ = this->declare_parameter("base_frame", std::string("base_link"));

// El feedback de pose se obtiene mediante:
tf = tf_buffer_->lookupTransform(map_frame_, base_frame_, tf2::TimePointZero);
```

El controlador realiza `lookupTransform(map_frame_, base_frame_, tf2::TimePointZero)` en cada tick de control. Si se pasa `base_frame:=base_link_ekf` al lanzar, el controlador cerrará el lazo sobre la pose estimada por el EKF **sin cambiar una sola línea de código**.

**Criterio de validacion:** revisar que `lazo_cerrado.launch.py` ya declara los argumentos `map_frame` y `base_frame` con valores por defecto sobreescribibles (confirmado: líneas 71-74 del launch file). No se requiere ninguna accion de codigo.

---

## Paso 2 — Garantizar TF inicial correcta

**Objetivo:** asegurar que el EKF arranca con un estado consistente con el frame `map` de CoppeliaSim, de forma que el árbol TF `map→base_link_ekf` esté disponible desde el primer instante.

**Prerequisito: TF map→base_link disponible al arranque.**

CoppeliaSim publica via el plugin `vrep_ros_interface` la TF `map→base_link_gt` (ground truth). Sin embargo, el inicializador del EKF necesita `map→base_link` (la odométrica compuesta `map→odom→base_link`). `mecanum_odometry` publica `odom→base_link`, pero nadie publica `map→odom`. Solución: agregar al launch compuesto un nodo `static_transform_publisher` que emita `map→odom` como identidad:

```python
Node(package='tf2_ros', executable='static_transform_publisher',
     arguments=['0','0','0','0','0','0','map','odom'],
     parameters=[{'use_sim_time': use_sim_time}])
```

Esto garantiza que `map→odom→base_link` esté disponible para el lookup del EKF en `t=0`. (En la sim, el robot arranca en el origen, así que la identidad es correcta.)

**Mecanismo de inicialización (ya implementado en Etapa A):**

El nodo `ekf_localizer` realiza un único `lookupTransform("map", "base_link", tf2::TimePointZero)` al recibir el primer mensaje de odometría. Extrae `(x, y, yaw)` y los usa como estado inicial del EKF:

```cpp
// En el callback de odometría, primer tick:
geometry_msgs::msg::TransformStamped tf_init =
    tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
x_(0) = tf_init.transform.translation.x;
x_(1) = tf_init.transform.translation.y;
x_(2) = tf2::getYaw(tf_init.transform.rotation);
initialized_ = true;
```

**Nota sobre la asuncion map≡odom en t=0:**

La escena `omni_ekf.ttt` de CoppeliaSim coloca al robot exactamente en el origen del frame `map` al inicio de la simulacion. Por lo tanto, en `t=0` se cumple que `map ≡ odom` (la TF `map→base_link` devuelve `(0, 0, 0)`). Esta coincidencia hace que inicializar el EKF desde la TF `map→base_link` sea equivalente a inicializarlo en `(0, 0, 0)`.

Esta asuncion **solo es valida al inicio de la simulacion**. En cuanto el EKF comienza a procesar mediciones de bearing y la odometria acumula drift, las frames `map` y `odom` divergen: el EKF corrige la pose en `map` mientras que la odometria pura deriva en `odom`. Esta es exactamente la diferencia que el informe debe cuantificar.

**Criterio de validacion:**

```bash
# Verificar que la TF inicial esta en el origen:
ros2 run tf2_ros tf2_echo map base_link_ekf
# Esperado en t=0: translation [0.000, 0.000, 0.000], rotation [0.000, 0.000, 0.000, 1.000]
```

---

## Paso 3 — Launch compuesto `seguimiento_ekf.launch.py`

**Objetivo:** lanzar los tres modulos (modelo holonomico, EKF localizer, controlador a lazo cerrado) con un unico comando, pasando `use_sim_time` via `LaunchConfiguration` en lugar de strings literales.

Crear el archivo en `ekf_localizer/launch/seguimiento_ekf.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        # Argumento global de reloj.
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Usar el reloj de CoppeliaSim (/clock).'
        ),

        # TF map→odom identidad: necesaria para que el EKF pueda hacer
        # lookupTransform("map", "base_link") al inicializarse.
        # mecanum_odometry publica odom→base_link pero nadie publica map→odom;
        # esta TF estática cierra la cadena. Válida porque el robot arranca en
        # el origen del frame map (garantizado por omni_ekf.ttt).
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_static_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # Modulo 1: modelo holonomico (odometria + publicacion de /cmd_vel).
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

        # Modulo 3: EKF localizer.
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

        # Modulo 2: controlador a lazo cerrado,
        # con base_frame apuntando al frame publicado por el EKF.
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
```

**Razon del uso de `LaunchConfiguration` en lugar de `'true'`:** pasar el string literal `'true'` al `IncludeLaunchDescription` lo convierte en un valor fijo y no propagable desde la linea de comandos. Usando `LaunchConfiguration('use_sim_time')` el argumento se resuelve en tiempo de lanzamiento y puede ser sobreescrito con `use_sim_time:=false` para pruebas fuera del simulador.

**Criterio de validacion:**

```bash
# Compilar el paquete (si el launch file es nuevo):
colcon build --packages-select ekf_localizer --symlink-install

# Verificar que la descripcion de lanzamiento se parsea sin errores:
ros2 launch ekf_localizer seguimiento_ekf.launch.py --show-args
```

---

## Paso 4 — Verificar arbol TF del sistema completo

**Objetivo:** confirmar que el arbol de transformaciones es consistente antes de iniciar el seguimiento.

```bash
# Con CoppeliaSim corriendo y los tres modulos activos:
ros2 run tf2_tools view_frames

# Inspeccionar el PDF generado (frames.pdf). El arbol unificado debe ser:
#
#   map                          (raiz unica)
#   ├── odom                     (TF estatica identidad: map_to_odom_static_tf)
#   │   └── base_link            (publicado por mecanum_odometry)
#   │       ├── front_laser      (TF estatica de CoppeliaSim)
#   │       └── [ruedas]
#   └── base_link_ekf            (publicado por ekf_localizer)
#
# IMPORTANTE: map y odom deben aparecer conectados (NO como dos arboles separados).
# La TF map→odom identidad (publicada por map_to_odom_static_tf en el launch)
# es lo que une el arbol. Sin ella, view_frames muestra dos raices independientes
# y lookupTransform("map", "base_link") falla.

# Verificar individualmente (todos deben responder sin error):
ros2 run tf2_ros tf2_echo map base_link_ekf    # EKF pose
ros2 run tf2_ros tf2_echo map base_link         # odometria via map→odom→base_link
ros2 run tf2_ros tf2_echo odom base_link        # solo odometria
```

**Criterio de validacion:** `view_frames` muestra UN SOLO arbol con raiz `map`. La TF `map→base_link_ekf` se actualiza a la frecuencia del EKF (~30 Hz). `ros2 run tf2_ros tf2_echo map base_link` responde (confirma que la cadena `map→odom→base_link` es navegable). Si `base_link_ekf` no aparece, revisar que `ekf_localizer` haya recibido al menos un mensaje de `/robot/odometry` y haya completado la inicializacion.

---

## Paso 5 — Correr el seguimiento y observar en RViz

**Objetivo:** verificar visualmente que el robot sigue la trayectoria cuadrada usando la pose del EKF como feedback, y que la elipse de covarianza decrece al detectar postes.

**Lanzamiento:**

```bash
# Terminal 1: CoppeliaSim con escena omni_ekf.ttt (play).

# Terminal 2: stack completo.
ros2 launch ekf_localizer seguimiento_ekf.launch.py

# Terminal 3: RViz con config preexistente (o configurar manualmente).
rviz2 -d $(ros2 pkg prefix ekf_localizer)/share/ekf_localizer/rviz/ekf_seguimiento.rviz
```

**Configuracion de RViz:**

| Display | Topico / Frame | Descripcion |
|---------|----------------|-------------|
| Odometry | `/robot/odometry` | Pose odometrica pura (frame `odom`) |
| Odometry | `/robot/odometry_ekf` | Pose EKF (frame `map`); activar elipse de covarianza |
| Odometry | `/robot/ground_truth` | Pose real del simulador |
| Path | `/real_path` | Trayectoria recorrida segun el controlador |
| PointCloud2 / LaserScan | `/laser/scan` | Escaneo laser (deteccion de postes) |
| PoseArray | `/posts` | Mapa de postes en frame `map` |
| TF | — | Arbol completo |

**Indicadores de correcto funcionamiento:**

- El robot sigue el cuadrado (waypoints en `±2 m, ±2 m`).
- La elipse de covarianza de `/robot/odometry_ekf` se contrae visiblemente al pasar cerca de los postes y se expande en trayectos sin observaciones.
- La trayectoria del EKF converge a la del ground truth mejor que la odometria pura tras completar al menos media vuelta.

**Criterio de validacion:** el controlador no se detiene (no imprime "Trayectoria completada" prematuramente) y el error de posicion al final de cada lado del cuadrado es menor con EKF que con odometria pura.

---

## Paso 6 — Rosbag de las tres estimaciones

**Objetivo:** capturar los topicos relevantes para la comparacion cuantitativa del informe.

```bash
# Crear directorio de grabaciones:
mkdir -p ~/rosbags/seguimiento_ekf

# Iniciar grabacion (en paralelo con el seguimiento):
ros2 bag record \
    /robot/odometry \
    /robot/ground_truth \
    /robot/odometry_ekf \
    /posts \
    /laser/scan \
    /clock \
    -o ~/rosbags/seguimiento_ekf/run_$(date +%Y%m%d_%H%M%S)
```

**Nota sobre topicos de pose EKF:**

El nodo `ekf_localizer` publica la estimacion como `nav_msgs/Odometry` en `/robot/odometry_ekf` (consistente con `/robot/odometry` del Modulo 1). Si la consigna exige adicionalmente un topico `geometry_msgs/PoseStamped` (por ejemplo, `/robot/pose_ekf`), agregar en el nodo un segundo publisher:

```cpp
// En ekf_localizer.cpp, junto al publisher de Odometry:
pose_ekf_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
    "/robot/pose_ekf", rclcpp::QoS(10));

// En publishState(), despues de publicar la Odometry:
geometry_msgs::msg::PoseStamped pose_msg;
pose_msg.header = odom_msg.header;
pose_msg.pose   = odom_msg.pose.pose;
pose_ekf_pub_->publish(pose_msg);
```

**Criterio de validacion:**

```bash
# Verificar que el bag contiene los tres topicos principales:
ros2 bag info ~/rosbags/seguimiento_ekf/run_*/
# Buscar en la lista: /robot/odometry, /robot/ground_truth, /robot/odometry_ekf
# Nota: /robot/pose_ekf NO existe en el nodo actual. Si se necesita ese topico,
# agregar un segundo publisher en ekf_localizer.cpp (ver bloque de codigo mas abajo).
```

---

## Paso 7 — Comparar trayectorias para el informe

**Objetivo:** producir las figuras y metricas que documentan la mejora del EKF sobre la odometria pura.

**Opcion A — Script Python con rosbag2\_py:**

```python
#!/usr/bin/env python3
"""Compara odometria pura, EKF y ground truth desde un rosbag."""
import sys
import numpy as np
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

BAG_PATH = sys.argv[1]

reader = rosbag2_py.SequentialReader()
storage_options = rosbag2_py.StorageOptions(uri=BAG_PATH, storage_id='sqlite3')
converter_options = rosbag2_py.ConverterOptions('', '')
reader.open(storage_options, converter_options)

topics = {
    '/robot/odometry':     'nav_msgs/msg/Odometry',
    '/robot/ground_truth': 'nav_msgs/msg/Odometry',
    '/robot/odometry_ekf': 'nav_msgs/msg/Odometry',
}
data = {t: [] for t in topics}

while reader.has_next():
    topic, raw, _ = reader.read_next()
    if topic in topics:
        msg_type = get_message(topics[topic])
        msg = deserialize_message(raw, msg_type)
        p = msg.pose.pose.position
        data[topic].append((p.x, p.y))

for label, topic in [('Odometria pura', '/robot/odometry'),
                      ('EKF',           '/robot/odometry_ekf'),
                      ('Ground truth',  '/robot/ground_truth')]:
    pts = np.array(data[topic])
    if len(pts):
        plt.plot(pts[:, 0], pts[:, 1], label=label)

plt.axis('equal')
plt.grid(True)
plt.legend()
plt.xlabel('X [m]')
plt.ylabel('Y [m]')
plt.title('Comparacion de trayectorias — Modulo 3')
plt.tight_layout()
plt.savefig('comparacion_trayectorias.png', dpi=150)
plt.show()
```

**Metricas cuantitativas sugeridas para el informe:**

- Error de posicion RMS al final de cada vuelta: `RMSE = sqrt(mean((x_est - x_gt)^2 + (y_est - y_gt)^2))`.
- Error maximo de posicion durante la vuelta completa.
- Reduccion porcentual de RMSE: EKF vs. odometria pura.
- Numero de postes detectados por vuelta y su correlacion con los valles de la norma de covarianza.

**Criterio de validacion:** la figura muestra que la trayectoria EKF se aproxima al ground truth mejor que la odometria pura, especialmente en la segunda mitad de la vuelta donde el drift odometrico es mayor.

---

## Riesgos

| Riesgo | Probabilidad | Mitigacion |
|--------|-------------|------------|
| TF `map→base_link_ekf` no disponible al arrancar el controlador | Media | El controlador ya maneja `TransformException` con `RCLCPP_WARN_THROTTLE` y reintenta en el siguiente tick; el EKF solo tarda unos ticks en inicializarse. |
| Latencia entre modulos causa lag en la TF | Baja | Usar `tf2::TimePointZero` en `lookupTransform` (ya implementado en el controlador) para obtener la ultima TF disponible. |
| `use_sim_time` no propagado correctamente | Baja | Usar `LaunchConfiguration` en el launch compuesto (no strings literales). Verificar con `ros2 param get /ekf_localizer use_sim_time`. |
| Inestabilidad del EKF por mala inicializacion de covarianza | Media | Si el robot oscila, aumentar `P0` diagonal. Si diverge por mediciones ruidosas, aumentar `R` (ruido de medicion del laser). |
| Rosbag demasiado grande | Baja | Grabar solo los topicos listados en el Paso 6; excluir `/rosout` y topics de debug. |
| El frame `base_link_ekf` no coincide con el nombre esperado por el controlador | Baja | Verificar con `ros2 run tf2_ros tf2_echo map base_link_ekf` antes de lanzar el controlador. |

---

## Estimacion

**3.5 horas totales**

| Tarea | Tiempo estimado |
|-------|----------------|
| Paso 1: revision de codigo del controlador | 15 min |
| Paso 2: verificacion de TF inicial y nota map≡odom | 20 min |
| Paso 3: escritura y depuracion del launch compuesto | 45 min |
| Paso 4: verificacion del arbol TF | 20 min |
| Paso 5: corrida con RViz y ajuste de parametros | 45 min |
| Paso 6: grabacion del rosbag | 30 min |
| Paso 7: script de comparacion y figuras para el informe | 35 min |
| Margen para imprevistos | 20 min |
| **Total** | **3 h 30 min** |
