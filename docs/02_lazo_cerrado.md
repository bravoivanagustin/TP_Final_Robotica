# Módulo 2: Lazo Cerrado (Seguimiento de Trayectoria)

Este documento describe el segundo módulo del TP — el nodo de **control a lazo cerrado con seguimiento de trayectoria** para el robot Mecanum — y la arquitectura del paquete `lazo_cerrado/`.

## Rol dentro del TP

La consigna §2 pide:

- Un **controlador proporcional (P) por cada GDL** que haga converger la pose del robot hacia una pose objetivo, usando la odometría (vía TF) como feedback.
- Seguir una **trayectoria cuadrada de 2 m de lado**, vértices `(±2, ±2)` en frame `map`, con orientación **opuesta al centro** (yaw deseado en cada punto = `atan2(y, x)`).
- **Pursuit-based goal selection**: definir waypoints intermedios, redefinir el objetivo según cercanía.
- Publicar `/robot/cmd_vel` (`geometry_msgs/Twist`) periódicamente.
- Justificar experimentalmente los valores de Kp.

El módulo se monta sobre el módulo 1 (`modelo_holonomico/mecanum_odometry`) que aporta la odometría y la TF `odom → base_link`. El simulador publica `map → odom`, así que `map → base_link` se compone vía TF. En el módulo 3, `base_link` será reemplazado por `base_link_ekf` — el controlador soporta el cambio sin recompilar vía el param `base_frame`.

## Arquitectura del paquete

Layout calcado del peer pedagógico `lazo_abierto_diferencial/`:

```
lazo_cerrado/
├── CMakeLists.txt
├── package.xml
├── include/lazo_cerrado/
│   ├── HolonomicTrajectoryFollower.h    # base abstracta del controlador
│   └── KinematicHolonomicController.h   # derivada: ley P + pursuit + TF feedback
├── src/
│   ├── HolonomicTrajectoryFollower.cpp
│   ├── KinematicHolonomicController.cpp
│   ├── trajectory_follower_node.cpp     # entrypoint del controlador
│   └── trajectory_generator_node.cpp    # entrypoint del generador del cuadrado
└── launch/
    └── lazo_cerrado.launch.py
```

Dos ejecutables ROS 2: `trajectory_follower` (controlador) y `trajectory_generator` (publica la trayectoria una vez con QoS TransientLocal). Una librería interna `trajectory_controller` con la base abstracta, linkeada por el follower.

### `HolonomicTrajectoryFollower` (base)

Refactor del `TrajectoryFollower` del taller diferencial (`lazo_abierto_diferencial/include/lazo_abierto/TrajectoryFollower.h`) con cuatro cambios:

| Aspecto | Diferencial | Holonómico (este módulo) |
|---|---|---|
| Tópico cmd | `/cmd_vel` | **`/robot/cmd_vel`** (lo que escucha `mecanum_odometry`) |
| Twist publicado | `linear.x = v`, `angular.z = w`, resto cero | `linear.x = vx`, `linear.y = vy`, `angular.z = wz` |
| Firma virtual | `control(t, double& v, double& w)` | `control(t, double& vx, double& vy, double& wz)` |
| Arranque del timer | Al recibir la primera trayectoria | **En el constructor**, a `control_rate_hz` (default 30 Hz). Así los modos sin trayectoria (FIXED) funcionan sin generator activo. |

Se mantienen: suscripción a `/robot/trajectory` con QoS Reliable + TransientLocal (el controlador se engancha al último mensaje aunque arranque después que el generator), helper `nextPointIndex(t, idx)` (no se usa en pursuit pero queda por simetría), virtual puro que devuelve `false` para terminar la trayectoria (la base cancela el timer y publica Twist=0 final).

### `KinematicHolonomicController` (derivada)

Implementa la ley de control y la selección de goal. Los componentes clave:

**Feedback por TF**: `tf2_ros::Buffer::lookupTransform(map_frame_, base_frame_, TimePointZero)`. Si la TF no está lista al arranque (sim arrancando), publica Twist=0 y reintenta cada tick. Cambiar `base_frame` a `base_link_ekf` desde el launch suficiente para usar la pose refinada del módulo 3.

**Ley P por GDL en frame body** (no Siegwart): el `(ρ, α, β)` del diferencial acopla heading con velocidad lineal — inservible cuando el robot debe strafear con orientación independiente.

```
ex_w = gx - x ; ey_w = gy - y ; eθ = normalize(gθ - θ)
ex_b =  cos(θ)·ex_w + sin(θ)·ey_w
ey_b = -sin(θ)·ex_w + cos(θ)·ey_w
vx = kp_xy · ex_b
vy = kp_xy · ey_b
wz = kp_yaw · eθ
```

No hay saturación de velocidades — la consigna no la pide, y los controladores del taller diferencial (`lazo_cerrado_diferencial/src/KinematicPositionController.cpp`, `lazo_abierto_diferencial/src/FeedForwardController.cpp`) tampoco la usan. Ver §"Idea pendiente" abajo si fuese necesaria en barridos de Kp altos.

**Modos de selección de goal** (param `goal_mode`):

- `FIXED`: usa `fixed_goal_{x,y,yaw}`. Para sanity checks (avance puro, strafing puro, rotación pura, combinado).
- `PURSUIT`: cursor monotónico `last_goal_idx_` sobre la trayectoria suscrita; arranca desde el waypoint más cercano al robot (no desde índice 0). Avanza mientras la distancia al wp actual ≤ `lookahead_distance`. Cuando el último wp queda a menos de `position_tolerance` y `yaw_tolerance` → devuelve `false` (la base cancela el timer).

**Visualización en RViz**: publica `/goal_pose` (`PoseStamped`) cada tick y `/real_path` (`nav_msgs/Path`) acumulativo cada ~10 ticks (≈3 Hz a 30 Hz de loop).

### `trajectory_generator_node`

Standalone (no hereda). Construye la trayectoria cuadrada en el `main()`:

- Vértices CCW: `(+s, +s) → (-s, +s) → (-s, -s) → (+s, -s) → (+s, +s)` con `s = square_half_side` (default 2.0).
- Discretización: `n_steps = ceil(seg_len / waypoint_step)` por lado (default `waypoint_step = 0.05` → 80 wp por lado → 321 totales con cierre).
- Yaw de cada wp: `atan2(y, x)` — orientación radial saliente, **continua a lo largo de cada lado** (la discontinuidad de `atan2` en `±π` no se cruza dentro de un lado porque ningún lado pasa por el origen).
- Publica una vez en `/robot/trajectory` (`robmovil_msgs/Trajectory`) y `/desired_path` (`nav_msgs/Path`) con QoS Reliable + TransientLocal.

El campo `velocity` de cada `TrajectoryPoint` queda en cero (no se usa en pursuit). `time_from_start` se llena con un `dt_dummy` monótono (no se usa, pero el campo es requerido por el `.msg`).

## Tópicos del módulo

| Tópico | Tipo | Dirección | QoS | Nota |
|---|---|---|---|---|
| `/robot/cmd_vel` | `geometry_msgs/Twist` | pub (base) | Reliable, KeepLast(10) | Lo consume `mecanum_odometry`. |
| `/robot/trajectory` | `robmovil_msgs/Trajectory` | pub (generator) / sub (base) | Reliable, **TransientLocal** | Latched: late subscribers reciben. |
| `/desired_path` | `nav_msgs/Path` | pub (generator) | Reliable, **TransientLocal** | Visualización RViz, latched. |
| `/goal_pose` | `geometry_msgs/PoseStamped` | pub (derivada) | Reliable, KeepLast(10) | Waypoint perseguido en cada tick. |
| `/real_path` | `nav_msgs/Path` | pub (derivada) | Reliable, KeepLast(10) | Pose real acumulada, ~3 Hz. |
| TF | — | sub (derivada) | — | `lookupTransform(map_frame, base_frame)`. |

## Parámetros

### `trajectory_generator`

| Nombre | Tipo | Default | Descripción |
|---|---|---|---|
| `square_half_side` | double | `2.0` | [m] Semi-lado del cuadrado. Vértices `(±s, ±s)`. |
| `waypoint_step` | double | `0.05` | [m] Paso de discretización. |
| `frame_id` | string | `map` | Frame en el header de Trajectory y Path. |
| `use_sim_time` | bool | `true` | Por launch. |

### `trajectory_follower` (base + derivada)

| Nombre | Tipo | Default | Descripción |
|---|---|---|---|
| `control_rate_hz` | double | `30.0` | Frecuencia del timer. |
| `goal_mode` | string | `PURSUIT` | `FIXED` (sanity checks) o `PURSUIT` (cuadrado). |
| `kp_xy` | double | `0.8` | Ganancia P de traslación (a tunear). |
| `kp_yaw` | double | `1.5` | Ganancia P de orientación (a tunear). |
| `lookahead_distance` | double | `0.30` | [m] |
| `position_tolerance` | double | `0.05` | [m] umbral fin de trayectoria. |
| `yaw_tolerance` | double | `0.05` | [rad] umbral fin de trayectoria. |
| `map_frame` | string | `map` | Frame del setpoint y del feedback. |
| `base_frame` | string | `base_link` | Cambiable a `base_link_ekf` en módulo 3. |
| `fixed_goal_x`, `fixed_goal_y`, `fixed_goal_yaw` | double | `2.0`, `2.0`, `π/4` | Goal del modo `FIXED`. |

## Compilación y ejecución

Desde dentro del container `ros2_robotica`, en `/root/ros2_ws`:

```bash
colcon build --packages-select lazo_cerrado
source install/setup.bash
```

### Sanity checks aislados (modo FIXED)

Cada test asume el simulador corriendo y el módulo 1 lanzado en otra terminal:

```bash
# Terminal A: simulador (escena coppeliaSim/omni_ekf.ttt en play)
# Terminal B:
ros2 launch modelo_holonomico odometry.launch.py
# Terminal C (cada test reemplaza esta línea):
ros2 launch lazo_cerrado lazo_cerrado.launch.py \
    goal_mode:=FIXED kp_xy:=0.5 kp_yaw:=1.0 \
    fixed_goal_x:=<x> fixed_goal_y:=<y> fixed_goal_yaw:=<yaw>
```

Tests recomendados (orden):

1. **Salida cero** — goal = `(0, 0, 0)`. `cmd_vel` debe ser cero.
2. **Avance puro** — goal = `(1, 0, 0)`. `vx>0`, `vy≈0`, `wz≈0`.
3. **Strafing puro** — goal = `(0, 1, 0)`. `vx≈0`, `vy>0`, `wz≈0`. **Test crítico**: valida que el pipeline holonómico funciona end-to-end (cinemática inversa Mecanum del módulo 1 + ley P body de éste). Es el mismo test que cierra el módulo 1 (ver `docs/01_modelo_holonomico.md` §"Cómo verificar el módulo end-to-end" step 6).
4. **Rotación pura** — goal = `(0, 0, π/2)`. `vx≈0`, `vy≈0`, `wz>0`.
5. **Combinación + orientación radial** — goal = `(2, 2, π/4)` (primer vértice del cuadrado). Esperar combinación coherente de los tres.

### Recorrido completo del cuadrado (modo PURSUIT)

```bash
ros2 launch lazo_cerrado lazo_cerrado.launch.py kp_xy:=0.8 kp_yaw:=1.5
```

En RViz (`coppeliaSim/tpfinal.rviz` + agregar displays `Path` para `/desired_path` y `/real_path`, y `Pose` para `/goal_pose`):
- `/desired_path` dibuja el cuadrado.
- `/real_path` traza el recorrido real del robot.
- `/goal_pose` se ve adelantarse al robot por la distancia `lookahead_distance`.
- El robot strafea por los lados rotando para mantenerse orientado radialmente.
- En las esquinas hay un cambio de yaw de 90° — zona crítica de tuning de `kp_yaw`.
- Al cerrar la vuelta llega a `(+2, +2, π/4)` dentro de tolerancia y se detiene (el timer se cancela; último Twist publicado es cero).

### Experimentos para el reporte

La consigna pide variar Kp lineal y Kp angular **por separado**, al menos 3 valores cada uno. Grabar un rosbag por configuración:

```bash
ros2 bag record -o experimentos/control_p/kp_xy_0.8_kp_yaw_1.5/bag \
    /robot/cmd_vel /robot/odometry /robot/trajectory \
    /goal_pose /desired_path /real_path \
    /tf /tf_static
```

Procesar offline con `rosbag2_py` + matplotlib: graficar `x` vs `y` (deseado vs real), `(x, y, θ)` vs `t`, y `(vx, vy, wz)` vs `t`. Ver `docs/REPORT.md` §3 para el contenido esperado del informe.

## Decisiones de diseño

### Por qué un `kp_xy` único (no `kp_x` y `kp_y` separados)

Mecanum es simétrico en `x`/`y` del cuerpo idealmente (mismos rodillos en las 4 ruedas, misma masa distribuida). La consigna pide variar **Kp lineal** vs **Kp angular**; un único `kp_xy` cubre lo pedido. Si experimentalmente se observa anisotropía (más fricción de rodillos en una dirección), se separa.

### Por qué no la ley Siegwart `(ρ, α, β)`

El controlador clásico del diferencial fuerza al robot a alinear su heading con la dirección de movimiento (acopla `v` lineal con θ). El TP holonómico exige lo contrario: strafing por los lados del cuadrado mientras la orientación apunta radialmente hacia afuera. La ley P por GDL desacoplada en frame body es la generalización natural para Mecanum.

### Por qué el timer arranca en el constructor (no al recibir la trayectoria)

El diferencial pedagógico arranca el timer en `handleNewTrajectory()`, lo que hace que el controlador no haga nada hasta que el generator publique. Para nosotros eso significa que el modo `FIXED` (sanity checks aislados, sin generator) no funcionaría. Arrancando el timer en el constructor el modo `FIXED` queda autosuficiente y el modo `PURSUIT` empieza a publicar Twist=0 mientras espera el primer mensaje del generator.

### Por qué el launcher usa `DeclareLaunchArgument` por cada parámetro

Diverge de los launchers viejos del taller diferencial (`lazo_abierto_diferencial/launch/lazo_abierto.launch.py`, `lazo_cerrado_diferencial/launch/lazo_cerrado.launch.py`) que hardcodean los valores dentro de `parameters=[...]`. Acá cada param está declarado con `DeclareLaunchArgument` + `default_value` + `description`, y se referencia con `LaunchConfiguration` adentro de `parameters=`. El costo es ~5× más líneas; el beneficio es que **los barridos experimentales de Kp lineal y angular que pide la consigna se hacen desde la línea de comandos sin tocar el `.py` ni recompilar**:

```bash
ros2 launch lazo_cerrado lazo_cerrado.launch.py kp_xy:=0.4 kp_yaw:=1.0
ros2 launch lazo_cerrado lazo_cerrado.launch.py kp_xy:=0.8 kp_yaw:=2.0
```

Convención del proyecto, ver `.claude/CLAUDE.md` §"Convención de launch files".

### Idea pendiente

**Feed-forward de velocidades**: el generator construye una `Trajectory.msg` con campos `velocity` y `acceleration` en cero. Si en algún momento se quiere agregar feed-forward (control mixto FF + P), esos campos están listos para llenar — para el cuadrado los `(vx_world, vy_world)` son constantes por lado.

**Saturación de velocidades**: la consigna no la pide y el patrón pedagógico no la usa, pero podría hacer falta si los barridos experimentales de Kp llegan a valores altos. Lejos del setpoint (error ≈ `(2, 2, π/4)` al arranque del cuadrado) con `kp_xy ≥ 1.5` los `(vx, vy)` comandados superan `2 m/s`, lo cual amplifica a > 100 rad/s por rueda (la cinemática inversa divide por `r = 0.05`); el simulador termina capando cada rueda por separado, lo que **rompe la coherencia entre las 4 ruedas** y hace que el robot real no se mueva como `(vx, vy, wz)`. Si esto aparece en los experimentos, agregar dos params (`vxy_max`, `wz_max`) y dos helpers:
- `clamp_abs(z, lim)` para `wz` (saturación escalar manteniendo signo).
- `clamp_norm(vx, vy, lim)` para `(vx, vy)` (saturación vectorial manteniendo dirección — `clamp_abs` por componente cambiaría la dirección del vector velocidad y desviaría al robot del setpoint).
