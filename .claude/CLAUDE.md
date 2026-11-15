# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project context

TP Final ("Seguimiento de trayectoria a lazo cerrado basado en EKF para un robot omnidireccional") for **Robótica Móvil, 2do cuatrimestre 2025**. Holonomic 4-wheel Mecanum mobile robot — 350 mm × 350 mm platform, 50 mm wheel radius, 500 encoder ticks/revolution, absolute encoders — simulated in CoppeliaSim and controlled via ROS 2 Humble. Project docs, comments, and commit messages are in Spanish — match that language when editing them.

Authoritative references (read these before architectural changes):
- `TP Final - Robot Omnidireccional.pdf` — assignment spec; defines the three required modules, the simulator's topic/frame interface, and the report structure
- `Kinematic Model Of Four Mecanum Wheeled Mobile Robot.pdf` — Taheri/Qiao/Ghaeminezhad 2015; spec mandates implementing **eqs. 19–26 with the parameter table on page 8**
- `herramientas_robotica/ros_docker.pdf` and `herramientas_robotica/pasos_docker` — Docker setup walkthroughs
- `docs/chronicle.md` — running project log (append entries here as work progresses)
- `docs/01_modelo_holonomico.md` — module 1 deep-dive: contract, bug fixes, validation, verification checklist
- `docs/REPORT.md` — guide for the final TP report (what each section needs + what evidence to capture during implementation)

## Required deliverables (from the spec)

The TP demands **three ROS 2 nodes** in addition to the message package and URDF:

1. **Modelo Cinemático** (`modelo_holonomico/`, **implemented and validated**)
   - Subscribes `/robot/cmd_vel` (`geometry_msgs/Twist`), publishes per-wheel velocity on `/robot/{front,rear}_{left,right}_wheel/cmd_vel` (`std_msgs/Float64`, rad/s)
   - Subscribes `/robot/encoders` (`robmovil_msgs/MultiEncoderTicks`)
   - Publishes odometry as `nav_msgs/Odometry` on `/robot/odometry` AND TF **`odom → base_link`**
   - **Status:** node `mecanum_odometry_node` implements Mecanum IK (eq. 20) + FK (eq. 21) with body-frame twist (REP-105). Validated against analytical circular trajectory (~3 cm error in 27 s of integration). **Strafing test (`vy ≠ 0`, `vx = wz = 0`) is the remaining canary** for the historical diff-drive bug — see `docs/01_modelo_holonomico.md` §"Cómo verificar el módulo end-to-end" step 6.
   - Pending refactor (noted in `mecanum_odometry.cpp:12-13`): lift `WHEEL_L`, `WHEEL_RADIUS`, `ENCODER_TICKS` from `#define`s to `ros2 param` so they can be tuned without recompiling.
2. **Control a lazo cerrado y seguimiento de trayectoria** (`lazo_cerrado/`, **implemented**)
   - Clase base `robmovil::HolonomicTrajectoryFollower` (`src/HolonomicTrajectoryFollower.{h,cpp}`): timer de control a 30 Hz, publica `/robot/cmd_vel`, suscribe `/robot/trajectory`.
   - Derivada `robmovil::KinematicHolonomicController` (`src/KinematicHolonomicController.{h,cpp}`): ley P por GDL en frame body, feedback por TF `map → base_link`. Modos: `FIXED` (goal único, para sanity) y `PURSUIT` (cursor monotónico sobre waypoints con lookahead). **Incluye fase de pre-alineación**: antes del pursuit, rota en el lugar hasta igualar el yaw del primer goal (elimina el pico inicial de `wz` que causa slip Mecanum).
   - Nodo `trajectory_generator` (`src/trajectory_generator_node.cpp`): publica una `robmovil_msgs/Trajectory` del cuadrado `(±square_half_side, ±square_half_side)` CCW, con yaw por waypoint = `atan2(y, x)` ("opuesta al centro").
   - Launchers: `launch/lazo_cerrado.launch.py` (controller + generator) y `launch/seguimiento_odom.launch.py` (combo con odometría del módulo 1 en un solo proceso).
   - Deep dive: `docs/02_lazo_cerrado.md`.
3. **Localización basada en EKF** (`ekf_localizer/`, **implemented**)
   - Dos sub-nodos: `landmark_detector_node` (clustering de `/robot/front_laser/scan` → `/landmarks` en frame robot) y `ekf_localizer_node` (EKF con Eigen, predict desde `/robot/odometry` y update desde `/landmarks` matcheados contra `/posts` del sim).
   - Publica TF `map → base_link_ekf` y `/robot/odometry_ekf`. El mapa de postes (16, Ø 0.1 m) es **a priori** desde `/posts` (`geometry_msgs/PoseArray` en frame `map`).
   - Launcher combo `launch/seguimiento_ekf.launch.py`: levanta odometría + landmark_detector + EKF + controller cerrando el lazo con `base_frame:=base_link_ekf`.
   - Deep dives: `docs/00_decisiones_modulo3.md`, `docs/03_ekf_localizer_plan.md`, `docs/03b_ekf_modelo_matematico.md`, `docs/modulo3_etapa_{A,B,C,D}_*.md`.

Package names suggested by `herramientas_robotica/pasos_docker` (`lazo_abierto`, `lazo_cerrado`, `imu_laser`, `robmovil_ekf`, `astar_planning`) appear to be leftovers from earlier talleres; only some map onto TP-Final deliverables. Confirm before creating skeletons.

## Simulator interface (CoppeliaSim, scene `coppeliaSim/omni_ekf.ttt`)

The simulator's `/vrep_ros_interface` plugin defines the contract your nodes must plug into:

- **Simulator publishes:** `/robot/encoders` (MultiEncoderTicks, ~25 Hz), `/robot/front_laser/scan` (LaserScan), `/posts` (PoseArray of 16 landmarks in `map`), `/tf`, `/vrep_time`
- **Simulator subscribes:** the four `/robot/{...}/cmd_vel` Float64 topics

**Frame chain owned by the simulator:**
```
map ──► odom ──► base_link_gt          (ground truth — debug only, never consume from control)
                base_link ──► front_laser
```
Module 1 fills in `odom → base_link`. Module 3 will add `map → base_link_ekf`. **Never broadcast `map → base_link` from student code** — the simulator already owns `map → odom`, and `base_link_gt` is reserved for ground-truth comparison.

## Kinematics — equations implemented

Square Mecanum platform, wheel radius `r`, half-widths `lx = ly = 0.175 m` (so `lx + ly = 0.35 m = 2 · WHEEL_L`), wheel index convention `1 = FL, 2 = FR, 3 = RL, 4 = RR` (matches `MultiEncoderTicks.ticks[0..3]`).

**Inverse (`cmd_vel` → ω_i)** — paper eq. 20, implemented in `mecanum_odometry.cpp:on_velocity_cmd`:
```
ω₁ = (1/r) · ( vₓ − v_y − (lₓ+l_y)·ω_z)
ω₂ = (1/r) · ( vₓ + v_y + (lₓ+l_y)·ω_z)
ω₃ = (1/r) · ( vₓ + v_y − (lₓ+l_y)·ω_z)
ω₄ = (1/r) · ( vₓ − v_y + (lₓ+l_y)·ω_z)
```

**Forward (encoder Δticks → body-frame velocity)** — paper eq. 21, implemented in `mecanum_odometry.cpp:on_encoder_ticks`:
```
vₓ  = (r/4) · ( ω₁ + ω₂ + ω₃ + ω₄)
v_y = (r/4) · (−ω₁ + ω₂ + ω₃ − ω₄)
ω_z = (r/4) · (−ω₁ + ω₂ − ω₃ + ω₄) / (lₓ + l_y)
```
Body-frame `(Δx, Δy)` is rotated by `θ` to integrate into the `odom` frame. Note the denominator for `Δθ` is `8·WHEEL_L = 4·(lx+ly)`, **not** `4·WHEEL_L` — the latter was the historical off-by-2 bug.

## Development environment: Docker is mandatory

All ROS 2 work runs inside the `ros2_robotica` container. The container bundles ROS 2 Humble, Gazebo, CoppeliaSim 4.7.0, and the `sim_ros2_interface` (CoppeliaSim ↔ ROS 2 bridge). The host repo bind-mounts into the container at `/root/ros2_ws/src/robotica`, so host-side edits are visible immediately.

From `ros_docker/`:
```bash
bash start-docker.sh build    # build image (first time / after Dockerfile changes)
bash start-docker.sh start    # create & start container (mounts repo, X11)
bash start-docker.sh open     # open a bash shell in the running container
bash start-docker.sh stop     # stop
bash start-docker.sh down     # stop and remove
bash start-docker.sh rebuild  # down + rebuild image
```

The bind-mount path in `start-docker.sh:72` is hard-coded to `/home/agustinbravo/Desktop/TP_Final_Robotica` — if cloning elsewhere, this needs editing.

**Multi-terminal model.** Simulator, RViz, and each ROS 2 node typically each need their own `bash start-docker.sh open` shell. **`source install/setup.bash` is required in every new terminal**, not just where `colcon build` ran. A typical session opens 3+ shells in parallel: `coppeliaSim.sh`, `rviz2`, and one or more `ros2 launch` / `ros2 run` invocations.

**Rootless Docker on lab computers.** Faculty machines forbid `sudo`; Docker runs rootless and each terminal must export:
```bash
export DOCKER_HOST=unix:///run/user/$(id -u)/docker.sock
# if /run/user/$(id -u)/docker.sock doesn't exist:
dockerd-rootless.sh &
```
Not relevant on a personal machine with normal Docker, but anything that depends on this env var should be assumed broken on lab hardware.

## Building and running

From `/root/ros2_ws` (NOT from package dirs):
```bash
colcon build --symlink-install                       # build everything
colcon build --packages-select modelo_holonomico     # build one package
source install/setup.bash                             # required in every new terminal
```
`--symlink-install` matters because the host repo is bind-mounted — symlinks let edits propagate without rebuilds for interpreted resources.

**Running module 1**: use the launch file rather than `ros2 run` so `use_sim_time:=true` is set automatically (without it, sim-time stamps don't align with the node's clock and you get tf2 extrapolation warnings):
```bash
ros2 launch modelo_holonomico odometry.launch.py
# override (rare, only for rosbags with wall-clock stamps):
ros2 launch modelo_holonomico odometry.launch.py use_sim_time:=false
```

## Convención de launch files

Los launch files del TP usan el patrón **`DeclareLaunchArgument` + `LaunchConfiguration`** para **todos** los parámetros que tenga sentido tunear desde la línea de comandos (no sólo `use_sim_time`). Cada arg se declara con `default_value` y `description`, y se referencia con `LaunchConfiguration('nombre')` adentro de `parameters=[{...}]`. Ejemplo canónico: `lazo_cerrado/launch/lazo_cerrado.launch.py`.

**Por qué este patrón y no el "hardcoded inline"** (que usan los launchers pedagógicos viejos como `lazo_abierto_diferencial/launch/lazo_abierto.launch.py` o `lazo_cerrado_diferencial/launch/lazo_cerrado.launch.py`): la consigna pide barridos experimentales (variar `kp_xy`, `kp_yaw`, etc. al menos 3 veces cada uno). Con el patrón declarativo el barrido es una línea de comandos por corrida:
```bash
ros2 launch lazo_cerrado lazo_cerrado.launch.py kp_xy:=0.8 kp_yaw:=1.5
```
sin editar el `.py` ni rebuildear. Bonus: `--show-args` documenta los args desde el propio launcher.

**Costo aceptado**: cada param aparece 3 veces (declaración, variable `LaunchConfiguration`, entrada en `parameters=`). Es ~5× más líneas que el patrón hardcoded; vale la pena para los parámetros que vamos a barrer en experimentos.

**Regla práctica**: declarar como `DeclareLaunchArgument` todo param que (a) cambie entre corridas de validación o experimentos del reporte, (b) tenga que mutar para activar/desactivar features (`goal_mode`, `enable_logger`, etc.), o (c) deba ser swappable sin recompilar (p.ej. `base_frame: base_link → base_link_ekf` cuando entre el módulo 3). Lo que es geometría fija o cableado interno puede quedar hardcoded.

## CoppeliaSim ↔ ROS 2 bridge setup (one-time, per container)

The `sim_ros2_interface` plugin must be patched to know about `robmovil_msgs` AND `geometry_msgs/PoseArray` (for `/posts`) before it can publish/subscribe them. Per `herramientas_robotica/pasos_docker` and the TP annex, inside the container:

- `/root/ros2_ws/src/sim_ros2_interface/meta/interfaces.txt` — append at minimum:
  ```
  robmovil_msgs/msg/EncoderTicks
  robmovil_msgs/msg/MultiEncoderTicks
  geometry_msgs/msg/PoseArray
  rosgraph_msgs/msg/Clock
  sensor_msgs/msg/Imu
  nav_msgs/msg/MapMetaData
  nav_msgs/msg/OccupancyGrid
  geometry_msgs/msg/PoseStamped
  ```
- `/root/ros2_ws/src/sim_ros2_interface/package.xml` — add `<depend>robmovil_msgs</depend>`
- `/root/ros2_ws/src/sim_ros2_interface/CMakeLists.txt` — add `find_package(robmovil_msgs REQUIRED)` and list `robmovil_msgs` inside `ament_target_dependencies`

Then: `colcon build --packages-select sim_ros2_interface`.

## EKF library setup (when implementing module 3)

The course's `EKFilter` template requires matrix dimensions at construction. For the omnidirectional model — control input is 3-DoF `[vₓ, v_y, ω_z]` instead of the differential-drive `[v, ω]` — the constructor in `robmovil_ekf::LocalizerEKF` must read:
```cpp
robmovil_ekf::LocalizerEKF::LocalizerEKF(void) : EKFilter(3, 3, 3, 2, 2)
```
(vs. the differential-drive default of `EKFilter(3, 2, 3, 2, 2)`). This is called out in the spec annex.

## Architecture

### Packages currently in the repo

- **`robmovil_msgs/`** — message definitions: `EncoderTicks`, `MultiEncoderTicks`, `Trajectory`, `TrajectoryPoint`, `Landmark`, `LandmarkArray`. Any package that publishes/subscribes these must `<depend>robmovil_msgs</depend>` and `find_package(robmovil_msgs REQUIRED)`.
  - **`MultiEncoderTicks.ticks` ordering** (from spec §"Actuadores y sensores", reaffirmed in `mecanum_odometry.cpp:67-70`): `[front_left, front_right, rear_left, rear_right]`. **This order is contractual** — if a scene ever publishes encoders in a different order, the forward kinematics breaks silently.
  - **`Landmark`** is `(range, bearing)` — relative-polar, designed for landmark detections expressed in the robot frame.
- **`modelo_holonomico/`** — single C++ node `mecanum_odometry_node`:
  - Class `robmovil::MecanumOdometry` in `src/mecanum_odometry.{h,cpp}` (entrypoint in `src/mecanum_odometry_node.cpp`).
  - Node name registered in ROS 2: `mecanum_odometry`.
  - Launch: `launch/odometry.launch.py` (sets `use_sim_time:=true`).
  - Deep dive: `docs/01_modelo_holonomico.md`.
- **`lazo_cerrado/`** — controller del módulo 2 (ver §Required deliverables #2):
  - Ejecutables `trajectory_follower` + `trajectory_generator`.
  - Launchers `lazo_cerrado.launch.py` (controller+generator) y `seguimiento_odom.launch.py` (combo con módulo 1).
  - Deep dive: `docs/02_lazo_cerrado.md`.
- **`ekf_localizer/`** — localización EKF del módulo 3 (ver §Required deliverables #3):
  - Ejecutables `landmark_detector_node` + `ekf_localizer_node`.
  - Launcher combo `seguimiento_ekf.launch.py`: odometría + detector + EKF + controller con `base_frame:=base_link_ekf`.
  - Deep dives: `docs/00_decisiones_modulo3.md`, `docs/03*_ekf_*.md`, `docs/modulo3_etapa_*.md`.
- **`URDF/`** — `nashe.urdf` plus Collada meshes for body, Hokuyo URG-04LX laser, and the four Swedish wheels.
- **`coppeliaSim/`** — `omni_ekf.ttt` (scene), `omni_ekf_v2.ttt` (alternativa sin decidir), `tpfinal.rviz` (RViz config). Binary, not built.
- **`informe/`** — informe final en TeX (`Informe TP Final.tex` + `UBA.png` para la carátula).
- **`ros_docker/`** — Dockerfile and host-side launch scripts.
- **`herramientas_robotica/`** — supporting course docs (Docker walkthrough, etc.).
- **`scripts/`** y **`experiments/`** — infra de corridas experimentales reproducibles (ver §"Experiments framework" abajo).

### Kinematics constants (`modelo_holonomico/src/mecanum_odometry.cpp:14-16`)
```
WHEEL_L       = 0.175 m   (half-distance between wheel centers; lx = ly assumed)
WHEEL_RADIUS  = 0.05  m
ENCODER_TICKS = 500       (ticks per wheel revolution; encoders are absolute)
```
`(lx + ly) = 2 · WHEEL_L = 0.35` is the grouping that appears in eqs. 20–24. Constants are duplicated as `#define`s; lifting them to `ros2 param` is a worthwhile refactor when next touching this file (the `IDEA pendiente` comment at `mecanum_odometry.cpp:12-13` flags this).

## Project doc layout

```
.claude/CLAUDE.md                        ← this file: orientation for Claude Code
docs/
├── chronicle.md                         ← running project log (append entries with dates)
├── REPORT.md                            ← guide for writing the final TP report
├── 01_modelo_holonomico.md              ← módulo 1: contrato, bugs, validación
├── 02_lazo_cerrado.md                   ← módulo 2: controller + generator + launchers
├── 00_decisiones_modulo3.md             ← módulo 3: decisiones de alto nivel
├── 03_ekf_localizer_plan.md             ← módulo 3: plan de implementación
├── 03b_ekf_modelo_matematico.md         ← módulo 3: derivación Jacobianos y update
├── modulo3_etapa_A_landmark_detector.md ← módulo 3 etapa A: clustering del laser
├── modulo3_etapa_B_modelado_ekf.md      ← módulo 3 etapa B: modelo + ecuaciones
├── modulo3_etapa_C_nodo_ekf.md          ← módulo 3 etapa C: implementación del nodo
└── modulo3_etapa_D_integracion.md       ← módulo 3 etapa D: launcher combo + validación
informe/
├── Informe TP Final.tex                 ← informe final (TeX)
└── UBA.png                              ← logo para la carátula
docs_locales/                            ← gitignored — notas personales fuera del repo
```

Referencias externas (en la raíz del repo, no en `docs/`):
- `TP Final - Robot Omnidireccional.pdf` — consigna del TP
- `Kinematic Model Of Four Mecanum Wheeled Mobile Robot.pdf` — Taheri 2015 (eqs. 19–26)
- `herramientas_robotica/ros_docker.pdf`, `herramientas_robotica/pasos_docker.txt` — setup Docker

El `docs/` es el registro durable de *qué es el proyecto*; `.claude/` es el registro durable de *cómo Claude ayuda con él*. Los deep-dives por módulo (`docs/0N_*.md`) viven en `docs/` y se linkean desde acá a medida que se escriben.

## Experiments framework

Infra para **corridas reproducibles**: cada experimento vive en `experiments/YYYY-MM-DD-<slug>/` con `manifest.yaml` (parámetros declarativos), `bag/` (`rosbag2` SQLite, gitignored), `plots/` (PNGs generados, versionados) y `notes.md`.

```
scripts/
├── new_experiment.sh       ← bootstrap de una corrida (crea folder + rellena manifest con sha/fecha)
├── record.sh               ← wrapper de `ros2 bag record` leyendo topics del manifest
├── publish_cmd_vel.py      ← driver open-loop (publica Twist por `duracion_s` en sim-time)
├── run_experiment.sh       ← orquesta: launch backend + record + driver + watchdog + cleanup
└── plot_run.py             ← lee bag → genera xy_path, body_velocity, odom_vs_cmd, wheel_velocities,
                              tracking_error, yaw_error, goal_pose_vs_pose. Overlay GT desde /tf.

experiments/
├── README.md               ← workflow para armar y correr corridas
├── template/
│   ├── manifest.yaml       ← schema: launch, trayectoria, cmd_source, duracion_maxima_s, topics_grabados
│   └── notes.md
└── YYYY-MM-DD-<slug>/      ← corridas concretas
    ├── manifest.yaml
    ├── bag/                ← gitignored (binario grande)
    ├── plots/              ← versionado
    └── notes.md
```

Flujo típico:
```bash
bash scripts/new_experiment.sh <slug> "descripción"
# editar experiments/YYYY-MM-DD-<slug>/manifest.yaml
bash scripts/run_experiment.sh experiments/YYYY-MM-DD-<slug>
python3 scripts/plot_run.py experiments/YYYY-MM-DD-<slug>
```

`cmd_source.tipo` en el manifest dispatcha el driver:
- `open_loop_twist` → `publish_cmd_vel.py` lee `trayectoria.parametros` y publica Twist.
- `closed_loop_launch` → `ros2 launch <paquete> <archivo> <args>` (típicamente `lazo_cerrado.launch.py`).
- `manual` → espera Enter del usuario (teleop, etc.).

**Disciplina de sim-time**: el sim corre a `simSpeed ≠ 1×`; `rosbag2` estampa con wall-clock por default. `publish_cmd_vel.py` fuerza `use_sim_time=True` y mide con `get_clock()`; `plot_run.py` usa `msg.header.stamp` (sim-time) para `dt` en cálculos de velocidad y bag-time para X del plot. Ver `scripts/plot_run.py:extract_wheel_encoder_velocities` para el patrón.

**Disciplina de ROS_DOMAIN_ID**: el `~/.bashrc` del container setea `ROS_DOMAIN_ID=12` y `ROS_LOCALHOST_ONLY=1`. Cualquier `docker exec ... bash -c "..."` que no cargue `.bashrc` queda en `DOMAIN_ID=0` y **no ve los tópicos del sim**. Exportar explícitamente al principio del comando o usar `bash -lc`.

## Intended Claude Code organization (per user, from `chronicle.md` 19/05/2026)

The user plans to grow the Claude-side scaffolding alongside the project:
- **One rule per module** to implement (likely under `.claude/rules/` or as separate CLAUDE.md sections).
- **One skill** to leverage code from earlier talleres with the differential-drive robot.
- **One session per module** (so each module's iteration stays focused).
- **One specialist** for the project overall ("Experto Proyecto" — the current session).

None of this is wired up yet; it's the intended trajectory. When opening a new session, check whether the user wants you to create / continue one of these specialized roles before defaulting to general help.

## Custom-message workflow

Adding a new `.msg` to `robmovil_msgs/msg/`:

1. Append the filename to `rosidl_generate_interfaces(...)` in `robmovil_msgs/CMakeLists.txt` (note: the legacy ROS 1 `add_message_files` block is commented out and unused).
2. If non-builtin types are referenced, add to the `DEPENDENCIES` argument and to `package.xml`.
3. Rebuild `robmovil_msgs` first, then any consumer packages.
4. If CoppeliaSim needs to see the new type, also add it to `sim_ros2_interface/meta/interfaces.txt` and rebuild that package.
