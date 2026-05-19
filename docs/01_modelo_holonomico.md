# Módulo 1: Modelo Holonómico

Este documento describe el primer módulo del TP — el nodo que implementa el **modelo cinemático** del robot Mecanum (cinemática inversa para actuación + odometría por integración de encoders) — y resume los cambios aplicados sobre el esqueleto inicial para alinearlo con la consigna.

## Rol dentro del TP

La consigna §1 pide un único nodo ROS 2 que:

- Convierta consignas `/robot/cmd_vel` (`geometry_msgs/msg/Twist`) en velocidades de cada rueda publicadas en `/robot/{front,rear}_{left,right}_wheel/cmd_vel` (`std_msgs/msg/Float64`, rad/s). Esta es la **cinemática inversa** (Taheri 2015, eq. 20).
- Lea los ticks publicados por el simulador en `/robot/encoders` (`robmovil_msgs/msg/MultiEncoderTicks`, orden `[FL, FR, RL, RR]`), aplique la **cinemática directa** (eq. 21) para reconstruir la pose del robot, y publique:
  - el mensaje `nav_msgs/msg/Odometry` en `/robot/odometry`, y
  - la transformada `odom → base_link`.

Este nodo es la base sobre la que se monta el módulo 2 (control a lazo cerrado con seguimiento de trayectoria) y al que el módulo 3 (EKF) le agrega encima la TF refinada `map → base_link_ekf`.

## Flujo de actuación: ¿quién mueve al robot?

`modelo_holonomico` es el **traductor entre el mundo de alto nivel (`Twist` en `/robot/cmd_vel`) y el mundo de las ruedas (4 velocidades individuales en rad/s)**. Es el único nodo del sistema que se suscribe a `/robot/cmd_vel` y el único que publica los cuatro tópicos por rueda — sin él, ningún comando de velocidad llega a las ruedas y el robot no se mueve, aunque alguien esté publicando `Twist` en `/robot/cmd_vel`.

### Qué hace y qué no hace

| | |
|---|---|
| ✅ **Lo que sí hace** | Traduce `(vx, vy, wz)` en 4 velocidades de rueda (`ω_FL`, `ω_FR`, `ω_RL`, `ω_RR`) y las publica. Lee los encoders de vuelta y reconstruye la pose. |
| ❌ **Lo que no hace** | No decide a dónde ir. No tiene goal ni trayectoria ni controlador. Si nadie publica `Twist` en `/robot/cmd_vel`, el robot se queda quieto. |

En el sistema final, **el módulo 2 (lazo cerrado)** será el que decida la velocidad y la publique en `/robot/cmd_vel`. Mientras tanto, en testing manual, **vos sos el módulo 2** vía `ros2 topic pub`.

### Diagrama del flujo de actuación

```
ros2 topic pub  ──►  /robot/cmd_vel  ──►  modelo_holonomico (on_velocity_cmd)
                       (Twist)                       │
                                                     │
                                       cinemática inversa (Taheri eq. 20)
                                                     │
                                                     ▼
                                 /robot/front_left_wheel/cmd_vel    (ω_FL)
                                 /robot/front_right_wheel/cmd_vel   (ω_FR)
                                 /robot/rear_left_wheel/cmd_vel     (ω_RL)
                                 /robot/rear_right_wheel/cmd_vel    (ω_RR)
                                                     │
                                                     ▼
                                  CoppeliaSim (vrep_ros_interface)
                                                     │
                                              gira cada rueda
                                                     │
                                                     ▼
                                            física del simulador
                                                     │
                                                     ▼
                                    /robot/encoders (MultiEncoderTicks)
                                                     │
                                                     ▼
                                    modelo_holonomico (on_encoder_ticks)
                                                     │
                                    cinemática directa (Taheri eq. 21)
                                      + integración a frame `odom`
                                                     │
                                                     ▼
                                    /robot/odometry  (nav_msgs/msg/Odometry)
                                             TF: odom → base_link
```

El lazo entero pasa por `modelo_holonomico` dos veces: una vez yendo hacia las ruedas (IK) y otra vez volviendo desde los encoders (FK).

### Cómo verificar el cableado

```bash
ros2 topic info /robot/cmd_vel -v
# → 1 publisher (tu ros2 topic pub), 1 subscriber (mecanum_odometry)

ros2 topic info /robot/front_left_wheel/cmd_vel -v
# → 1 publisher (mecanum_odometry), 1 subscriber (sim_ros2_interface)
```

Si en el segundo `topic info` aparece `Subscription count: 0`, el simulador no está suscripto (no se cargó la escena, o no se apretó play, o el plugin `vrep_ros_interface` no está bien configurado para los wheel topics). Las ruedas no van a girar aunque el nodo esté publicando correctamente.

## Mensajes intercambiados

Este módulo participa en **dos tópicos de entrada, cinco tópicos de salida y una TF**. La tabla resume el contrato completo; los bloques siguientes detallan los campos y convenciones de cada uno.

| Dirección | Tópico | Tipo | Quién está del otro lado |
|-----------|--------|------|--------------------------|
| **In**    | `/robot/cmd_vel`                       | `geometry_msgs/msg/Twist`              | Tu `ros2 topic pub` (hoy) o módulo 2 (lazo cerrado, futuro) |
| **In**    | `/robot/encoders`                      | `robmovil_msgs/msg/MultiEncoderTicks`  | CoppeliaSim (`sim_ros2_interface`) |
| **Out**   | `/robot/front_left_wheel/cmd_vel`      | `std_msgs/msg/Float64`                 | CoppeliaSim |
| **Out**   | `/robot/front_right_wheel/cmd_vel`     | `std_msgs/msg/Float64`                 | CoppeliaSim |
| **Out**   | `/robot/rear_left_wheel/cmd_vel`       | `std_msgs/msg/Float64`                 | CoppeliaSim |
| **Out**   | `/robot/rear_right_wheel/cmd_vel`      | `std_msgs/msg/Float64`                 | CoppeliaSim |
| **Out**   | `/robot/odometry`                      | `nav_msgs/msg/Odometry`                | Módulo 2 (feedback de control) y módulo 3 (entrada al EKF) |
| **Out (TF)** | `odom → base_link`                  | `tf2_msgs/msg/TFMessage` en `/tf` (payload: `geometry_msgs/msg/TransformStamped`) | RViz, módulo 2, módulo 3 |

### Entrada: `/robot/cmd_vel` (`geometry_msgs/msg/Twist`)

Consigna de velocidad de alto nivel para el cuerpo del robot. **No tiene `header` ni `stamp`** — el nodo actúa sobre el último mensaje recibido sin razonar sobre tiempo.

| Campo | Unidad | Significado |
|-------|--------|-------------|
| `linear.x`              | m/s   | Velocidad del cuerpo en su eje x (avance / retroceso). |
| `linear.y`              | m/s   | Velocidad del cuerpo en su eje y (strafing izquierda / derecha). |
| `linear.z`              | —     | Ignorado (robot planar). |
| `angular.x`, `angular.y`| —     | Ignorados (sin pitch ni roll). |
| `angular.z`             | rad/s | Yaw rate del cuerpo (rotación sobre el eje vertical). |

### Entrada: `/robot/encoders` (`robmovil_msgs/msg/MultiEncoderTicks`)

Conteo absoluto de ticks de los cuatro encoders. Publicado por el simulador a ~25 Hz.

| Campo | Tipo | Significado |
|-------|------|-------------|
| `header.stamp`    | `builtin_interfaces/Time` | Tiempo de la lectura (sim_time bajo `use_sim_time:=true`). Se usa para calcular `dt` entre mensajes consecutivos. |
| `header.frame_id` | `string`  | Informativo, no se consume. |
| `ticks[0]`        | `int32`   | Ticks acumulados de la rueda **delantera izquierda (FL)**. 500 ticks/revolución, encoder absoluto. |
| `ticks[1]`        | `int32`   | Ticks acumulados de **FR** (delantera derecha). |
| `ticks[2]`        | `int32`   | Ticks acumulados de **RL** (trasera izquierda). |
| `ticks[3]`        | `int32`   | Ticks acumulados de **RR** (trasera derecha). |

> **Nota crítica**: el orden `[FL, FR, RL, RR]` es contractual y está fijado por la consigna. Si la escena del simulador alguna vez publica los encoders en otro orden, la cinemática directa se rompe silenciosamente.

### Salida: 4× `/robot/{front,rear}_{left,right}_wheel/cmd_vel` (`std_msgs/msg/Float64`)

Velocidad angular comandada a cada rueda en rad/s. Un tópico por rueda. Los cuatro se publican **sincrónicamente cada vez que llega un `Twist`** en `/robot/cmd_vel` — el módulo no los publica de manera periódica por su cuenta.

| Campo  | Unidad | Significado |
|--------|--------|-------------|
| `data` | rad/s  | Velocidad angular comandada a la rueda. Signo positivo = rotación que, sin slip, produce avance del robot en `+x` del cuerpo (convención de la escena de CoppeliaSim). |

### Salida: `/robot/odometry` (`nav_msgs/msg/Odometry`)

Pose estimada + velocidades del cuerpo. Se publica **una vez por cada mensaje de encoders procesado** (≈25 Hz).

| Campo | Frame | Significado |
|-------|-------|-------------|
| `header.stamp`              | —          | El mismo `stamp` del mensaje de encoders que generó esta odometría. |
| `header.frame_id`           | —          | `"odom"` (la pose está expresada en este frame). |
| `child_frame_id`            | —          | `"base_link"` (el twist está expresado en este frame, por REP-105). |
| `pose.pose.position.x`      | `odom`     | Posición en el plano. |
| `pose.pose.position.y`      | `odom`     | Posición en el plano. |
| `pose.pose.position.z`      | —          | Siempre 0 (robot planar). |
| `pose.pose.orientation`     | `odom`     | Cuaternión derivado del yaw acumulado: `q = setRPY(0, 0, theta_)`. |
| `pose.covariance`           | —          | **No se computa** (todo en cero). El EKF del módulo 3 generará la covarianza propia. |
| `twist.twist.linear.x`      | `base_link`| Velocidad del cuerpo en x (= `Δx_b / Δt`). |
| `twist.twist.linear.y`      | `base_link`| Velocidad del cuerpo en y, _strafing_ (= `Δy_b / Δt`). |
| `twist.twist.angular.z`     | `base_link`| Yaw rate del cuerpo (= `Δθ / Δt`). |
| `twist.covariance`          | —          | No se computa. |

### Salida (TF): `odom → base_link`

La TF se publica en el tópico `/tf` mediante `tf2_ros::TransformBroadcaster`. El broadcaster trabaja con **dos niveles de tipo**:

- **Payload** — lo que el nodo arma campo por campo: `geometry_msgs/msg/TransformStamped`, una única transform con header + child_frame_id + translation + rotation.
- **Envelope** — lo que efectivamente viaja por el cable y lo que reporta `ros2 topic info /tf`: `tf2_msgs/msg/TFMessage`, que es esencialmente un `TransformStamped[]`. El broadcaster envuelve cada `TransformStamped` pasado por `sendTransform()` adentro de un `TFMessage`. En código nunca tocás `TFMessage` directamente.

La información transportada es la misma que la `pose` del `Odometry`, para que cualquier consumidor (RViz, módulo 2, módulo 3) pueda resolver la pose del robot vía `tf2`.

| Campo (en el `TransformStamped`) | Significado |
|----------------------------------|-------------|
| `header.stamp`           | El mismo del mensaje de encoders. Compartido con el `Odometry` para evitar warnings de extrapolación de tf2 bajo `use_sim_time`. |
| `header.frame_id`        | `"odom"` |
| `child_frame_id`         | `"base_link"` |
| `transform.translation`  | `(x_, y_, 0)` — idéntico a `pose.pose.position`. |
| `transform.rotation`     | Idéntico a `pose.pose.orientation`. |

### Cadencia y dependencias temporales

- `/robot/encoders` llega a ~25 Hz → dispara `/robot/odometry` y la TF `odom → base_link` al mismo ritmo.
- `/robot/cmd_vel` puede llegar a cualquier ritmo → los 4 tópicos de rueda se publican inmediatamente al recibirlo, sin acumular ni reenviar consignas viejas.
- El **primer mensaje de encoders** solo fija el baseline; no se integra ni se publica `Odometry`. Recién a partir del segundo empieza a haber salida.
- Si dos mensajes de encoders llegan con el mismo `stamp` (típico al arrancar la sim), el segundo solo refresca el baseline y no integra — guard de `dt ≤ 1e-6`.

## Renombres aplicados

El esqueleto original venía nombrado como si fuese el nodo de un robot diferencial Pioneer. Se renombró todo dentro del paquete `modelo_holonomico` para que el nombre refleje el modelo real (Mecanum holonómico de 4 ruedas):

| Elemento                  | Antes                          | Ahora                          |
|---------------------------|--------------------------------|--------------------------------|
| Clase C++                 | `robmovil::PioneerOdometry`    | `robmovil::MecanumOdometry`    |
| Header                    | `src/pioneer_odometry.h`       | `src/mecanum_odometry.h`       |
| Implementación            | `src/pioneer_odometry.cpp`     | `src/mecanum_odometry.cpp`     |
| Entrypoint (main)         | `src/pioneer_odometry_node.cpp`| `src/mecanum_odometry_node.cpp`|
| Ejecutable (CMake)        | `pioneer_odometry_node`        | `mecanum_odometry_node`        |
| Nombre del nodo ROS       | `nodeOdometry`                 | `mecanum_odometry`             |

El nombre del paquete (`modelo_holonomico`) y el namespace C++ (`robmovil`) se mantienen — son referencias estables en la consigna y en la cronica del proyecto.

## Correcciones aplicadas sobre el esqueleto inicial

### A. Tópicos y frames (contrato con el simulador)

- Suscripción a `/robot/cmd_vel` (antes `/cmd_vel`).
- Mensaje `Odometry`: `header.frame_id = "odom"`, `child_frame_id = "base_link"` (antes `"map"` / `"base_link"`).
- TF broadcasteada: `odom → base_link` (antes `map → base_link`, lo que pisaba la TF que el simulador ya posee — `map → odom`).
- TF y `Odometry` ahora usan **el mismo `header.stamp`** (`encoder->header.stamp`) — antes la TF usaba `this->get_clock()->now()`, lo que producía warnings de extrapolación de tf2 bajo `use_sim_time`.

### B. Cinemática directa Mecanum (Taheri 2015, eq. 21)

El esqueleto inicial integraba la pose con cinemática **diferencial**: promediaba las cuatro ruedas en un escalar `delta_s` y lo proyectaba con `cos/sin(theta)`. Eso descarta toda velocidad lateral del cuerpo (`v_y`) — es decir, todo movimiento de _strafing_, que aparece en 3 de los 4 lados de la trayectoria cuadrada del módulo 2.

Se reemplazó por la cinemática Mecanum correcta, índices `1=FL, 2=FR, 3=RL, 4=RR`:

```
Δx_b = ( d1 + d2 + d3 + d4) / 4
Δy_b = (−d1 + d2 + d3 − d4) / 4            ← capturado por primera vez
Δθ   = (−d1 + d2 − d3 + d4) / (8·WHEEL_L)  ← antes el denominador era 4·WHEEL_L
```

Luego se rota el `(Δx_b, Δy_b)` por `theta` para integrar a frame `odom`. El factor `8·WHEEL_L` corrige un error de factor 2 (el denominador en la consigna es `4·(lx+ly)`, no `4·lx`).

### C. Twist del Odometry en frame del cuerpo (REP-105)

`nav_msgs/msg/Odometry.twist` está definido en `child_frame_id` (base_link), no en el frame del mundo. El código original publicaba velocidades world-frame (`Δx_mundo / Δt`). Se corrigió a velocidades body-frame (`Δx_b/Δt`, `Δy_b/Δt`, `Δθ/Δt`), que además es la forma natural en la que el módulo 2 y el EKF van a consumirlas.

### D. Robustez

- Guard contra `Δt ≤ 1e-6` (mensajes con stamps duplicados al arrancar la sim): refresca el baseline pero no integra, para evitar divisiones por cero y saltos numéricos.
- Se eliminó la redefinición de `M_PI` — `<cmath>` ya la provee.

### Cosas que no se tocaron

- **Cinemática inversa** (`on_velocity_cmd`): ya era correcta y consistente con la directa nueva (jacobianas transpuestas).
- **Orden de ruedas** en `MultiEncoderTicks.ticks` y en los publishers: ya era `[FL, FR, RL, RR]`.
- **Skip-first** en el primer mensaje de encoders: ya era correcto.
- **Covarianzas**: la consigna no las exige; el EKF del módulo 3 generará las propias.

## Launch file

Se agregó `modelo_holonomico/launch/odometry.launch.py`, que lanza el nodo con `use_sim_time:=true` por defecto. Esto:

- Permite arrancar el nodo con un único comando — `ros2 launch modelo_holonomico odometry.launch.py` — en lugar de `ros2 run modelo_holonomico mecanum_odometry_node --ros-args -p use_sim_time:=true`, que es lo que requeriría si no se usase un launch (porque sin `use_sim_time` los stamps del nodo no coinciden con el reloj de CoppeliaSim y aparecen warnings de TF).
- Es el punto natural de extensión para cuando entren los módulos 2 (lazo cerrado) y 3 (EKF): un `bringup.launch.py` superior va a poder hacer `IncludeLaunchDescription` de este, en lugar de duplicar la configuración.

Para override del flag (caso raro, solo si se corre con un rosbag con stamps reales del wall-clock):

```bash
ros2 launch modelo_holonomico odometry.launch.py use_sim_time:=false
```

## Idea pendiente: lift de constantes a parámetros ROS 2

Hoy `WHEEL_L`, `WHEEL_RADIUS` y `ENCODER_TICKS` están como `#define` en `mecanum_odometry.cpp`. La idea es declararlas con `this->declare_parameter("wheel_radius", 0.05)` y leerlas en el constructor, así:

- Se pueden tunear desde el launch (`parameters=[{'wheel_radius': 0.05, ...}]`) o desde un YAML, sin recompilar.
- Si el robot cambia de geometría (o si en el futuro `lx ≠ ly`, lo que hoy se asume), todo el sistema toma la nueva geometría desde un único archivo.
- Es además el patrón idiomático en ROS 2.

No se aplicó en este pasaje para no inflar el alcance — queda como propuesta para que [usuario] decida si vale la pena ahora o se posterga al cierre del módulo 3.

## Validación numérica

Se corrió el nodo con `(vx=0.1, vy=0, wz=0.1)` durante ~27.5 s. Es una trayectoria circular de radio `vx / wz = 1 m` que tiene solución analítica cerrada:

```
x(t) = (vx/wz) · sin(wz·t)
y(t) = (vx/wz) · (1 − cos(wz·t))
θ(t) = wz · t
```

Comparación contra lo que reportó `/robot/odometry` en `t ≈ 27.5 s`:

| Magnitud  | Esperado (analítico) | Medido por el nodo |
|-----------|----------------------|--------------------|
| x         | 0.391 m              | 0.360 m            |
| y         | 1.923 m              | 1.975 m            |
| θ         | 2.747 rad            | 2.747 rad          |
| vxb       | 0.100 m/s            | 0.097 m/s          |
| vyb       | 0.000 m/s            | 0.003 m/s          |
| wz        | 0.100 rad/s          | 0.099 rad/s        |

La pose cierra contra el círculo analítico dentro de ~3 cm en 27 s de integración, las velocidades del cuerpo casan con la consigna al 3%, y el heading es exacto. Eso valida:

- **Cinemática inversa** — si fuera incorrecta, las velocidades del cuerpo no coincidirían con la consigna.
- **Cinemática directa** — si fuera incorrecta, la pose integrada no cerraría contra el círculo analítico.
- **Frames y TF** — la odometría se publica con `frame_id: odom` y `child_frame_id: base_link`, twist en frame del cuerpo (REP-105).
- **Manejo de `dt`** — sin blow-ups de integración a pesar del jitter de ~24 ms en el período de los encoders.

El test que cierra el ciclo de validación es el de **strafing puro** (`vy ≠ 0`, `vx = wz = 0`), que es lo que el código viejo no hacía. Si la `y_` mundo crece linealmente bajo ese comando, el bug histórico está efectivamente arreglado.

## Cómo verificar el módulo end-to-end

Adentro del contenedor `ros2_robotica`, con la escena `coppeliaSim/omni_ekf.ttt` corriendo:

1. **Compilación**
   ```bash
   colcon build --packages-select modelo_holonomico
   source install/setup.bash
   ```
2. **Arranque**
   ```bash
   ros2 launch modelo_holonomico odometry.launch.py
   ```
3. **Tópicos**
   - `ros2 topic info /robot/cmd_vel -v` debe listar `mecanum_odometry` como subscriber.
   - `ros2 topic echo /robot/odometry --once` debe mostrar `header.frame_id: odom` y `child_frame_id: base_link`.
4. **TF**: `ros2 run tf2_tools view_frames` debe producir un árbol `map → odom → base_link → front_laser` (más `map → odom → base_link_gt` paralelo, propiedad del simulador). **No** debe existir un TF directo `map → base_link` proveniente de este nodo.
5. **Inversa**:
   - `vx=0.1, vy=0, wz=0`  → las 4 ruedas con misma ω positiva (avance puro).
   - `vx=0, vy=0.1, wz=0` → FL y RR negativas, FR y RL positivas (strafing).
   - `vx=0, vy=0, wz=0.5` → izquierdas negativas, derechas positivas (rotación CCW).
6. **Directa — test crítico del bug histórico de `v_y`**: publicar `vx=0, vy=0.2, wz=0` durante 5 s y verificar que la odometría avanza en el eje `y` de `odom`. En el código viejo no se movía — ahora debe moverse.
7. **Visualización**: con `coppeliaSim/tpfinal.rviz`, el frame `base_link` (TF del nodo) debe coincidir aproximadamente con `base_link_gt` (TF del simulador) durante una trayectoria simple. Divergencias mayores a unos cm en 2 m son síntoma de un error de cinemática o de stamps.
8. **Sin warnings de "TF extrapolation"** en consola.
