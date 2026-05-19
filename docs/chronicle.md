# Cronica del Proyecto

La idea de este archivo es relatar los cambios que se hicieron en el proyecto de manera cronologica y ordenada. Esto se inicio posteriormente al inicio del proyecto.

### 18/05/2026

Inicio de la cronica, hasta ahora en el proyecto está implementado modelo_holonomico nomas, es la herramienta con la cual logramos controlar al robot holonomico. La idea es ir completando con los modulos necesarios alrededor para tener la estructura que requiere el TP. 

Este mismo necesita revisión pero es un buen primer avance

### 19/05/2026

/init con Claude Code, por ahora vamos a tener CLAUDE.md, a mi me gustaria tener una rule por cada modulo a implementar, una skill para aprovechar el codigo de los talleres con el otro robot y una sesion para cada modulo y una especialista en el proyecto en si. 

Trabajo sobre `modelo_holonomico` para alinearlo con la consigna del TP. Lo hecho hoy:

- **Renombre completo** dentro del paquete: `pioneer_odometry` → `mecanum_odometry` (clase, archivos en `modelo_holonomico/src/`, ejecutable en `CMakeLists.txt`, nombre del nodo ROS). El paquete sigue llamándose `modelo_holonomico`.
- **Cuatro bugs corregidos** en `modelo_holonomico/src/mecanum_odometry.cpp`: tópico `/cmd_vel` → `/robot/cmd_vel`, frames `map` → `odom`, denominador angular `4·L` → `8·L`, integración diferencial → Mecanum (Taheri eq. 21, captura `v_y`). Además: twist en frame del cuerpo (REP-105), guard contra `dt=0`, stamp consistente entre Odometry y TF.
- **Archivos nuevos**: `modelo_holonomico/launch/odometry.launch.py` (lanza el nodo con `use_sim_time:=true`) y `docs/01_modelo_holonomico.md` (doc completa del módulo: rol, diagrama del flujo de actuación `ros2 topic pub` → ruedas → encoders → odometría, tabla de renombres, correcciones, idea pendiente de levantar constantes a parámetros ROS 2, validación numérica contra trayectoria circular analítica, y checklist de verificación end-to-end).
- **Dep agregada** en `modelo_holonomico/package.xml`: `tf2_geometry_msgs` (ya estaba en `CMakeLists.txt`).
- **Referencia actualizada** en `docs/REPORT.md:50` al nombre nuevo del archivo.
- **Validación en simulador**: con `(vx=0.1, vy=0, wz=0.1)` la odometría cierra contra el círculo analítico de radio 1 m con error de ~3 cm en 27 s. El test crítico que falta correr para validar el fix del bug histórico es **strafing puro** (`vy ≠ 0`, `vx = wz = 0`).

Para retomar el módulo 1: leer `docs/01_modelo_holonomico.md`.

Implementación inicial del módulo 2 (`lazo_cerrado`) en una sesión guiada por plan. Lo hecho hoy:

- **Paquete nuevo `lazo_cerrado/`** con dos ejecutables (`trajectory_follower`, `trajectory_generator`) y una librería interna (`trajectory_controller`). Build limpio: `colcon build --packages-select lazo_cerrado` cierra sin warnings.
- **Arquitectura** calcada del taller diferencial (`lazo_abierto_diferencial/`): clase base abstracta `HolonomicTrajectoryFollower` + derivada `KinematicHolonomicController` (deriva e implementa `control(t, vx, vy, wz)`). Cambios sobre la base diferencial: publisher `/cmd_vel` → `/robot/cmd_vel`, firma 2 GDL → 3 GDL, arranque del timer en el constructor (no al recibir trayectoria) para que el modo `FIXED` funcione sin generator activo.
- **Ley de control**: P por GDL en frame body (`ex_b = cos·ex_w + sin·ey_w`, idem y rot.; `vx = kp_xy·ex_b`, `vy = kp_xy·ey_b`, `wz = kp_yaw·eθ`). **Sin Siegwart `(ρ, α, β)`** porque acopla heading con velocidad lineal y rompería la orientación radial del cuadrado. Sin saturación de velocidades (el patrón pedagógico no la usa; queda como idea pendiente en `docs/02_lazo_cerrado.md` si en los barridos de Kp altos aparecen problemas).
- **Feedback por TF** `map → base_link` (cambiable a `base_link_ekf` cuando entre el módulo 3 sin recompilar, vía param `base_frame`). Con TF no lista al arranque, el controlador publica Twist=0 y reintenta.
- **Pursuit goal selection** con cursor monotónico (`last_goal_idx_`) y arranque desde el waypoint más cercano. Una sola vuelta — al llegar a `(+2, +2, π/4)` dentro de `position_tolerance` y `yaw_tolerance`, el `control()` devuelve `false` y la base cancela el timer + publica Twist=0 final.
- **Generador del cuadrado**: nodo standalone `trajectory_generator` que construye 321 waypoints CCW desde `(+2, +2)` con yaw radial `atan2(y, x)` y publica `/robot/trajectory` + `/desired_path` con QoS TransientLocal (latched).
- **Visualización**: el controlador publica `/goal_pose` cada tick y `/real_path` acumulativo cada ~10 ticks. Pensado para sumar a la config de RViz existente (`coppeliaSim/tpfinal.rviz`) con displays `Path` y `Pose`.
- **Launch file** (`lazo_cerrado.launch.py`) que lanza generator + follower, con todos los args útiles para experimentar (`kp_xy`, `kp_yaw`, `lookahead_distance`, `goal_mode`, params del cuadrado, params del modo FIXED).
- **Rename del paquete viejo**: `lazo_cerrado_diferencial/package.xml` y `CMakeLists.txt` ahora declaran `<name>lazo_cerrado_diferencial</name>` / `project(lazo_cerrado_diferencial)`. Antes ambos paquetes querían el mismo nombre `lazo_cerrado` y `colcon` no los podía indexar a la vez. Código fuente del diferencial intacto: queda como referencia.
- **Doc nueva**: `docs/02_lazo_cerrado.md` con rol, arquitectura, tópicos, params, ley de control, batería de tests `FIXED` recomendados (avance puro, strafing puro, rotación pura, combinado), recorrido `PURSUIT` y flujo de experimentos con rosbag para el reporte.

**Pendiente para una próxima sesión** (no bloqueante para el módulo en sí):

- Correr la batería de sanity checks `FIXED` en el simulador. Especialmente el strafing puro `(0, 1, 0)` que sigue siendo el test crítico del módulo 1 también.
- Tunear `kp_xy` y `kp_yaw` con rosbags para los gráficos del reporte (al menos 3 valores de cada uno, ver `docs/REPORT.md` §3).
- Agregar displays `Path` y `Pose` a `coppeliaSim/tpfinal.rviz` para `/desired_path`, `/real_path` y `/goal_pose`.

Para retomar el módulo 2: leer `docs/02_lazo_cerrado.md`.

