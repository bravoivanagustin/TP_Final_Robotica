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

### Setup de terminales (al arrancar sesión)

Abrir **4 terminales** del container (`bash ~/Desktop/TP_Final_Robotica/ros_docker/start-docker.sh open` en cada una). En cada terminal, antes de cualquier comando:

```bash
cd /root/ros2_ws && source install/setup.bash
```

Roles fijos:

- **T1** — CoppeliaSim: `coppeliaSim.sh`, después `File → Open scene…` → `/root/ros2_ws/src/robotica/coppeliaSim/omni_ekf.ttt`, después Play (▶).
- **T2** — módulo 1: `ros2 launch modelo_holonomico odometry.launch.py`.
- **T3** — módulo 2: cambia entre tests (cada test es un `ros2 launch lazo_cerrado lazo_cerrado.launch.py …`).
- **T4** — inspección: `ros2 topic echo`, `ros2 topic hz`, `tf2_echo`, eventualmente `rviz2`.

### Verificación inicial del pipeline (al arrancar sesión)

Con **T1** (sim en play) y **T2** (módulo 1) corriendo, antes de tocar el controlador validar desde **T4** que el pipeline base está sano. Tres comandos, en este orden:

#### 1. Encoders publicándose

```bash
ros2 topic hz /robot/encoders
```

**Qué descubrís**:

- **Esperado por `CLAUDE.md`**: ~25 Hz.
- **Observado en el entorno actual** (verificado en sesión `Experto Lazo Cerrado`): **~6.4 Hz** en wall clock, con jitter alto (`std_dev ≈ 0.085 s`, `min ≈ 0.055 s`, `max ≈ 0.30 s`).
- Diferencia explicable: CoppeliaSim corriendo a ~25 % de tiempo real. **No es bug** — la lógica del controlador usa sim time, sólo significa que los experimentos toman ~4× más wall clock. Si se quiere acelerar, ajustar el `dt` o el factor de tiempo real desde la toolbar del simulador.

**Bloqueante si**: `ros2 topic hz` dice `no new messages`. Causa: sim no está en Play, o no se cargó la scene correcta. Acción: volver a T1, cargar `omni_ekf.ttt`, apretar ▶.

#### 2. Odometría publicándose

```bash
ros2 topic hz /robot/odometry
```

**Qué descubrís**: misma frecuencia que `/robot/encoders` (~6.4 Hz en este entorno). Es 1-a-1 porque `mecanum_odometry` publica un `Odometry` por cada `MultiEncoderTicks` recibido.

**Bloqueante si**: encoders publican pero odometry no. Causa: el módulo 1 no arrancó o crasheó. Mirar logs de T2.

#### 3. TF `map → base_link` disponible

```bash
ros2 run tf2_ros tf2_echo map base_link
```

**Qué descubrís** (en el entorno actual):

- Al arrancar, un warning de ~1 seg: `Invalid frame ID "map" passed to canTransform argument target_frame`. **Normal** — race condition mientras el simulador publica el frame `map` por primera vez.
- Después, salida estable cada ~300 ms:
  ```
  At time <sim_time>
  - Translation: [-2.005, 1.995, 0.060]
  - Rotation: yaw ≈ 1.586 rad ≈ 90.85°
  ```
- **Hallazgo importante**: **el robot NO arranca en el origen.** Arranca en `(x₀, y₀, θ₀) ≈ (-2.005, +1.995, π/2)` — la esquina superior-izquierda del cuadrado de la consigna, mirando hacia world +y.
  - Como `θ₀ = π/2`: **body +x (adelante) = world +y**; **body +y (izquierda) = world -x**.
  - Los goals de los sanity checks tienen que computarse en función de esta pose inicial, no de `(0, 0, 0)`. Las secciones de cada Test abajo ya traen los valores correctos.

**Bloqueante si**: queda en `Waiting for transform map -> base_link` por más de varios segundos. Causa: sim no está en play, o módulo 1 no está publicando `odom → base_link`.

### Sanity checks aislados (modo FIXED)

Cada test asume **T1** y **T2** corriendo. **T3** se relanza con cada test cambiando los `fixed_goal_*`. **T4** se usa para `ros2 topic echo /robot/cmd_vel` y observar lo que publica el controlador.

Todos los tests usan `kp_xy:=0.5 kp_yaw:=1.0` salvo donde se aclare — son valores conservadores para distancias chicas (~0.5 m). El Test 5 baja `kp_xy` porque cubre una distancia grande.

#### Reset cycle entre tests

El módulo 1 acumula `x_, y_, theta_` en memoria. Si se resetea la sim pero no se reinicia el módulo 1, la odometría arranca chueca (los encoders saltan a 0, el módulo 1 calcula un delta enorme). Por eso entre cada test hay que hacer el ciclo completo:

1. **T3**: `Ctrl+C` — matar el follower.
2. **T2**: `Ctrl+C` — matar el módulo 1.
3. **T1** (CoppeliaSim): Stop (⏹) → Play (▶).
4. **T2**: relanzar `ros2 launch modelo_holonomico odometry.launch.py`.
5. **T4**: verificar con `ros2 run tf2_ros tf2_echo map base_link` que la pose vuelva a `≈ (-2.005, +1.995, π/2)`.
6. **T3**: lanzar el siguiente test.

Es tedioso pero único modo de tener bien definida la pose inicial en cada corrida.

#### Test 1 — Salida cero (`goal = pose actual`)

**Objetivo**: confirmar que con error cero la ley P publica `cmd_vel ≈ 0`. Smoke test más barato del pipeline.

**Comando** (T3):

```bash
ros2 launch lazo_cerrado lazo_cerrado.launch.py \
    goal_mode:=FIXED fixed_goal_x:=-2.005 fixed_goal_y:=1.995 fixed_goal_yaw:=1.586 \
    kp_xy:=0.5 kp_yaw:=1.0
```

**Qué observar visualmente** (T1): el robot no se mueve.

**Qué observar en T4** (`ros2 topic echo /robot/cmd_vel`):

```
linear:  x: ~0.0     y: ~0.0     z: 0.0
angular: x: 0.0      y: 0.0      z: ~0.0
```

Pequeño ruido (`< 0.01`) es esperable — la odometría del módulo 1 tiene drift que el controlador intenta compensar.

✅ **Pasa si**: el robot queda quieto y `cmd_vel` tira valores ≈ 0.

❌ **Falla si**:

- El robot se mueve → la TF apunta a otro lado del que se tipeó como goal. Posible reset incompleto, posible problema con el frame `map`.
- `cmd_vel` no aparece en topic echo → el follower no arrancó (ver logs T3) o el timer no corre.

→ Reset cycle, siguiente test.

#### Test 2 — Avance puro (body +x, equivale a world +y)

**Objetivo**: validar que la rotación del error al frame body anda y que el módulo 1 sabe avanzar puro.

Como `θ₀ = π/2`, body +x apunta a world +y. Para mover **adelante** 0.5 m sumamos `+0.5` a `y₀`.

**Comando** (T3):

```bash
ros2 launch lazo_cerrado lazo_cerrado.launch.py \
    goal_mode:=FIXED fixed_goal_x:=-2.005 fixed_goal_y:=2.5 fixed_goal_yaw:=1.586 \
    kp_xy:=0.5 kp_yaw:=1.0
```

**Qué observar visualmente** (T1): el robot se mueve hacia adelante (visualmente, hacia world +y) **sin rotar**, hasta detenerse cerca de `(-2.005, 2.5)`.

**Qué observar en T4**:

```
linear:  x: 0.1–0.25   y: ~0.0      z: 0.0
angular: x: 0.0        y: 0.0       z: ~0.0
```

`linear.x > 0`, `linear.y ≈ 0`, `angular.z ≈ 0`.

✅ **Pasa si**: llega a `y ≈ 2.5` manteniendo `x ≈ -2.005` y `yaw ≈ π/2`. Durante la convergencia `cmd_vel.linear.x` decrece suavemente.

❌ **Falla si**:

- Se mueve diagonal o hacia otro lado → la rotación de error al body frame está mal (`cos(θ)·ex_w + sin(θ)·ey_w` no anda).
- Pivota como diferencial sin trasladarse → bug en el módulo 1.

→ Reset cycle, siguiente test.

#### Test 3 — **Strafing puro** (body +y, equivale a world -x) — **el crítico**

**Objetivo**: validar que el pipeline holonómico **completo** funciona — cinemática inversa Mecanum del módulo 1 + ley P body de éste. Es el test pendiente de validación del módulo 1 (ver `docs/01_modelo_holonomico.md` §"Cómo verificar el módulo end-to-end" step 6).

Como `θ₀ = π/2`, body +y apunta a world -x. Para strafear **a la izquierda del robot** 0.5 m restamos `0.5` de `x₀`.

**Comando** (T3):

```bash
ros2 launch lazo_cerrado lazo_cerrado.launch.py \
    goal_mode:=FIXED fixed_goal_x:=-2.5 fixed_goal_y:=1.995 fixed_goal_yaw:=1.586 \
    kp_xy:=0.5 kp_yaw:=1.0
```

**Qué observar visualmente** (T1): el robot **se desliza de costado** hacia world -x **sin rotar el chasis**. Las 4 ruedas giran en patrón Mecanum (FL+RR en un sentido, FR+RL en el opuesto), pero la posta es que el chasis se desplaza lateralmente sin pivotar.

**Qué observar en T4**:

```
linear:  x: ~0.0    y: 0.1–0.25   z: 0.0
angular: x: 0.0     y: 0.0        z: ~0.0
```

`linear.x ≈ 0`, `linear.y > 0`, `angular.z ≈ 0`.

✅ **Pasa si**: llega a `x ≈ -2.5` manteniendo `y ≈ 1.995` y `yaw ≈ π/2`, sin pivotar.

❌ **Falla si**:

- **El robot pivota en lugar de strafear** → bug del módulo 1 que no aplica el componente `vy` de la cinemática inversa. Es el bug histórico del modelo diferencial; este test lo verifica. Antes de seguir, mirar `modelo_holonomico/src/mecanum_odometry.cpp:50-53` y confirmar que la fila de `vy` esté presente.
- Se mueve diagonal → la rotación de error al body frame está mal.
- Se queda quieto → el plugin `vrep_ros_interface` no está enganchado a los 4 tópicos por rueda. Chequear `ros2 topic info /robot/front_left_wheel/cmd_vel -v` (Subscription count debería ser ≥ 1).

→ Reset cycle, siguiente test.

#### Test 4 — Rotación pura (`+π/2` desde la orientación actual)

**Objetivo**: validar que la ley P sobre `eθ` y la cinemática inversa para `wz` puro funcionan.

`θ₀ = π/2`. Sumamos `+π/2`: goal yaw = `π ≈ 3.1416`.

**Comando** (T3):

```bash
ros2 launch lazo_cerrado lazo_cerrado.launch.py \
    goal_mode:=FIXED fixed_goal_x:=-2.005 fixed_goal_y:=1.995 fixed_goal_yaw:=3.1416 \
    kp_xy:=0.5 kp_yaw:=1.0
```

**Qué observar visualmente** (T1): el robot rota **en el lugar** hasta apuntar a `π` rad (180°, mirando world -y). Las 4 ruedas giran en patrón Mecanum: las dos del lado izquierdo en un sentido, las del derecho en el opuesto.

**Qué observar en T4**:

```
linear:  x: ~0.0    y: ~0.0     z: 0.0
angular: x: 0.0     y: 0.0      z: 0.3–1.0
```

`linear.x ≈ 0`, `linear.y ≈ 0`, `angular.z > 0`.

✅ **Pasa si**: el yaw final es ≈ π (chequear con `tf2_echo`), y la posición se mantiene cerca de `(-2.005, 1.995)`.

❌ **Falla si**:

- El robot se traslada al rotar → bug en el módulo 1 (acoplamiento `wz → vx/vy` calculado mal en la cinemática inversa).

→ Reset cycle, siguiente test.

#### Test 5 — Combinado al siguiente vértice del cuadrado

**Objetivo**: combinar traslación + rotación con una distancia significativa. Ensayo general antes del PURSUIT.

El robot está en `(-2, +2)`. El siguiente vértice del cuadrado CCW es `(-2, -2)`. Orientación radial en `(-2, -2)`: `atan2(-2, -2) ≈ -3π/4 ≈ -2.356`.

Bajamos `kp_xy` a `0.3` porque el goal está a 4 m de distancia — con `0.5` puede haber overshoot fuerte (no hay saturación, ver §"Idea pendiente").

**Comando** (T3):

```bash
ros2 launch lazo_cerrado lazo_cerrado.launch.py \
    goal_mode:=FIXED fixed_goal_x:=-2.0 fixed_goal_y:=-2.0 fixed_goal_yaw:=-2.356 \
    kp_xy:=0.3 kp_yaw:=1.0
```

**Qué observar visualmente** (T1): el robot **baja por el lado izquierdo del cuadrado** (de `(-2, +2)` a `(-2, -2)`) mientras rota progresivamente desde `π/2` hasta `-3π/4`. Es exactamente lo que hace en uno de los lados del PURSUIT.

**Qué observar en T4**: combinación de `linear.x`, `linear.y`, `angular.z` no nulos; magnitudes decrecientes a medida que el robot se acerca al goal.

✅ **Pasa si**: llega cerca de `(-2, -2)` con yaw ≈ `-3π/4`. Pequeño overshoot u oscilación es aceptable con Kp bajo.

⚠️ **Comportamiento esperado pero llamativo**: con `kp_xy = 0.3` y error inicial de 4 m, `cmd_vel.linear.y` inicial es `0.3 × 4 = 1.2 m/s`. Eso son ~24 rad/s por rueda — alto pero dentro del rango que el simulador maneja. Si aparecen oscilaciones crecientes, bajar a `kp_xy:=0.2`.

❌ **Falla si**: la rotación se desacopla del avance y termina en una pose muy distinta a `(-2, -2, -3π/4)`.

→ Reset cycle. Si llegaste hasta acá, el módulo está sano y se puede pasar al PURSUIT.

### Recorrido completo del cuadrado (modo PURSUIT)

**Objetivo**: validar que el cuadrado completo se ejecuta con la mecánica de pursuit (cursor monotónico + lookahead + fin de trayectoria con tolerancia).

**Pre-condición**: que los Tests 1 y 3 hayan pasado al menos. Los demás son opcionales pero ayudan a debuggear si algo falla.

**Comando** (T3):

```bash
ros2 launch lazo_cerrado lazo_cerrado.launch.py kp_xy:=0.8 kp_yaw:=1.5
```

(defaults: `goal_mode:=PURSUIT`, `lookahead_distance:=0.30`, `square_half_side:=2.0`, `waypoint_step:=0.05`.)

**Setup adicional** (T4) — RViz para ver la trayectoria:

```bash
rviz2 -d /root/ros2_ws/src/robotica/coppeliaSim/tpfinal.rviz
```

En la GUI de RViz, **agregar manualmente tres displays** (Add → Path / Pose):

- `Path` con tópico `/desired_path` — color gris/blanco, espesor fino.
- `Path` con tópico `/real_path` — color brillante (verde, magenta), espesor grueso.
- `Pose` con tópico `/goal_pose` — para ver el waypoint perseguido.
- Confirmar Fixed Frame = `map`.

**Qué descubrís en este entorno**:

- El robot arranca en `(-2, +2, π/2)`. La orientación radial saliente en ese punto sería `atan2(2, -2) ≈ 2.356 rad ≈ 135°`, no `90°` — hay un **error inicial de yaw de ~45°** que el controlador corrige en los primeros segundos. Esperable, no es bug.
- El pursuit elige como primer goal el waypoint más cercano (cerca de `(-2, +2)`), no el primero del vector. Eso evita que el robot recorra el cuadrado al revés si arranca lejos del wp 0.

**Qué observar visualmente** (T1 + RViz):

- `/desired_path` dibuja el cuadrado completo en RViz.
- `/real_path` va trazando el recorrido real del robot (publish a ~3 Hz, no continuo).
- `/goal_pose` se ve adelantarse al robot por `lookahead_distance` = 0.30 m.
- El robot **strafea por los lados** rotando para mantener la orientación radial.
- En cada esquina (cambio de lado del cuadrado): cambio brusco de ~90° en el yaw deseado → rotación más marcada, posible overshoot moderado. Zona crítica de tuning de `kp_yaw`.
- Al cerrar la vuelta en `(+2, +2, π/4)` dentro de `position_tolerance` + `yaw_tolerance`, el log dice `"Trayectoria completada en (...)"`, el timer se cancela, último Twist publicado es cero.

**Qué observar en T4** (`ros2 topic echo /robot/cmd_vel`):

- En los lados rectos: `linear.x` y `linear.y` no nulos (porque el robot strafea), `angular.z` chico y de signo constante (gira lentamente para mantener yaw radial).
- En las esquinas: salto en `angular.z` cuando el waypoint perseguido pasa al lado siguiente.
- Al final: ráfaga de Twists ≈ 0 y luego silencio.

✅ **Pasa si**: el robot recorre el cuadrado completo y se detiene cerca de `(+2, +2)` con yaw ≈ `π/4`. La traza de `/real_path` sigue al cuadrado con error visible pero acotado, sin grandes desvíos.

❌ **Falla común**:

- Robot oscila o sale despedido al arrancar → `kp_xy` demasiado alto para el error inicial al primer wp. Bajar a `kp_xy:=0.5` o `kp_xy:=0.3` para la primera corrida; tunear después.
- Robot no llega a la primera esquina → algún lado del cuadrado no tiene waypoints. Chequear `ros2 topic echo --once /robot/trajectory | head -50` (debería listar 321 waypoints).
- Robot completa los 4 lados pero no termina (no aparece "Trayectoria completada") → no está alcanzando la tolerancia. Subir `position_tolerance:=0.1` o `yaw_tolerance:=0.1`.

→ Stop sim (no hace falta reset cycle si vas a relanzar PURSUIT con otros Kp inmediatamente).

### Experimentos para el reporte (barrido de Kp)

La consigna §2 pide **variar Kp lineal y Kp angular por separado, al menos 3 valores cada uno**. Mínimo 6 corridas (las dos del Kp central coinciden, así que 5 únicas + 1 control compartido). Grabar un rosbag por corrida para post-procesar.

**Plan de corridas sugerido**:

| Corrida | `kp_xy` | `kp_yaw` | Hipótesis |
|---|---|---|---|
| 1 | `0.4` | `1.5` | Respuesta traslacional lenta. El robot se queda atrás del setpoint. |
| 2 | `0.8` | `1.5` | Valor central de `kp_xy`. Punto de partida razonable. |
| 3 | `1.6` | `1.5` | Respuesta rápida. Overshoot probable en esquinas, oscilaciones. |
| 4 | `0.8` | `0.5` | Yaw lento. No alcanza a girar 90° antes del próximo wp. Yaw desfasado del radial. |
| 5 | (igual a 2) | — | Control compartido entre ambos barridos. Grabar una sola vez. |
| 6 | `0.8` | `3.0` | Yaw rápido. Oscilación angular en las esquinas. |

**Procedimiento por corrida**:

1. **Reset cycle** completo (kill T3, kill T2, Stop+Play sim, relanzar T2, verificar pose ≈ `(-2.005, 1.995, π/2)`).
2. **T4** — grabar bag (cambiar la carpeta por cada corrida):
   ```bash
   mkdir -p experimentos/control_p/kp_xy_0.4_kp_yaw_1.5
   cd experimentos/control_p/kp_xy_0.4_kp_yaw_1.5
   ros2 bag record -o bag \
       /robot/cmd_vel /robot/odometry /robot/trajectory \
       /goal_pose /desired_path /real_path \
       /tf /tf_static
   ```
3. **T3** — lanzar el follower con los Kp de la corrida:
   ```bash
   ros2 launch lazo_cerrado lazo_cerrado.launch.py kp_xy:=0.4 kp_yaw:=1.5
   ```
4. Esperar hasta el log `"Trayectoria completada"` (≈ 60–120 s wall clock con la sim a 25 %).
5. **T3**: `Ctrl+C` (el follower ya no publica, pero conviene matarlo). **T4**: `Ctrl+C` al bag.

**Procesamiento offline** (no es parte del módulo): un script tipo `scripts/plot_kp_sweep.py` con `rosbag2_py` + `matplotlib` que produzca, por cada bag:

- Plot cenital `x` vs `y`: `/desired_path` (gris fino) + `/real_path` (color grueso) + cruces en `/goal_pose`.
- 3 subplots `(x, y, θ)` vs `t`: setpoint (de `/goal_pose`) vs medido (de `/robot/odometry`).
- 3 subplots `(vx, vy, wz)` vs `t` de `/cmd_vel`.

**Métricas a reportar** (tabla en `docs/REPORT.md` §3):

- Error cuadrático medio de posición: `sqrt(mean((x_real - x_goal)² + (y_real - y_goal)²))`.
- Error máximo de yaw en las esquinas (zona peor).
- Tiempo total de la vuelta (en sim time).
- Overshoot por esquina (cuánto se pasa del vértice).

### Troubleshooting

| Síntoma | Causa probable | Acción |
|---|---|---|
| `tf2_echo map base_link` queda en `Waiting for transform…` | Sim no está en Play, o módulo 1 no arrancó | Verificar T1 (▶) y logs de T2. |
| `/robot/cmd_vel` publica pero el robot no se mueve | Plugin `vrep_ros_interface` no escucha los tópicos por rueda | `ros2 topic info /robot/front_left_wheel/cmd_vel -v` debe mostrar Subscription count ≥ 1. Si dice 0, problema del plugin de la sim. |
| Robot se mueve pero hacia el lugar equivocado | TF `map → base_link` apunta a otro frame, o reset incompleto entre tests | `tf2_echo map base_link` y comparar contra la pose visible en CoppeliaSim. Hacer reset cycle completo. |
| Nodo crashea al arrancar | `param` mal escrito en la línea de comandos | Mirar logs en T3; ROS 2 reporta los typos. |
| Frecuencia de encoders < 5 Hz | CoppeliaSim sobrecargado o `dt` excesivamente bajo | Toolbar de CoppeliaSim → simulation settings → ajustar `dt` o factor de tiempo real. |
| PURSUIT no termina (no aparece "Trayectoria completada") | El robot no alcanza el último waypoint con tolerancia | Subir `position_tolerance:=0.1` o `yaw_tolerance:=0.1`. |
| PURSUIT termina demasiado rápido (a los pocos segundos) | El first goal aterrizó cerca del último wp por casualidad | Resetear sim y relanzar. Posible mejora futura: requerir mínimo de waypoints recorridos antes de habilitar el "fin de trayectoria". |

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
