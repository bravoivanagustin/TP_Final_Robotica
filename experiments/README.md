# experiments/

Registro reproducible de corridas experimentales del TP Final. Cada subdirectorio `YYYY-MM-DD_<slug>/` es una corrida con su manifiesto, bag, plots y notas.

## Objetivo

Producir evidencia citable para el informe final: cada figura o número que aparezca en el reporte debería ser rastreable a una corrida acá adentro, con su git SHA y su configuración exacta. Sirve también para regresión — repetir el mismo experimento tras un cambio y comparar los plots.

## Flujo de tres pasos

**1. Crear la corrida** (host, en la raíz del repo):

```bash
bash scripts/new_experiment.sh strafing_canary "canary de strafing puro post-fix off-by-2"
```

Crea `experiments/YYYY-MM-DD_strafing_canary/` con `manifest.yaml` prellenado (slug, fecha, git SHA, dirty flag, descripción). **Editá el manifest a mano** — completá `launch`, `trayectoria`, `cmd_source`, `duracion_maxima_s`, y ajustá `topics_grabados` si hace falta (para módulo 2 sumar `/robot/trajectory`, `/goal_pose`, `/real_path`).

**2. Correr** (dentro del contenedor Docker, con `source install/setup.bash` hecho, y el simulador ya levantado en otra terminal):

```bash
bash scripts/run_experiment.sh experiments/2026-07-13_strafing_canary
```

Este script lanza backend + record + driver + watchdog, y flushea el bag prolijamente cuando el driver termina. Es el camino default para todas las corridas del TP.

**3. Plotear**:

```bash
python3 scripts/plot_run.py experiments/2026-07-13_strafing_canary
```

Los plots dependen del tipo de driver:
- `open_loop_twist` (módulo 1): `xy_path.png`, `body_velocity.png`, `odom_vs_cmd.png`.
- `closed_loop_launch` (módulo 2/3): agrega `tracking_error.png`, `yaw_error.png`, `goal_pose_vs_pose.png`.

## Anatomía del manifest

Los tres bloques que hay que completar a mano:

- **`launch`** — el "backend" (odometría, EKF). Ejemplo: `{paquete: modelo_holonomico, archivo: odometry.launch.py, args: {use_sim_time: true}}`.
- **`trayectoria`** — descripción de la referencia. La usan `plot_run.py` (overlays, tracking-error) y `publish_cmd_vel.py` en open-loop. Ejemplo: `{tipo: strafing, parametros: {vy: 0.2, duracion_s: 5.0}}`.
- **`cmd_source`** — quién publica `/robot/cmd_vel`:
  - `open_loop_twist` (módulo 1): `run_experiment.sh` lanza `publish_cmd_vel.py`, que lee `trayectoria` y publica un `Twist` fijo por `parametros.duracion_s` segundos.
  - `closed_loop_launch` (módulo 2/3): `run_experiment.sh` lanza `ros2 launch <paquete> <archivo> <args>`. Requiere `cmd_source.{paquete, archivo, args}`. El controller termina solo cuando alcanza el último waypoint.
  - `manual`: `run_experiment.sh` imprime instrucciones y espera Enter — para teleop u otros drivers ad-hoc.
- **`duracion_maxima_s`** — watchdog global. Si el driver no termina antes, `run_experiment.sh` mata todo. En open-loop, sea > `parametros.duracion_s`; en closed-loop, sea ≥ tiempo esperado de la trayectoria.

## Layout de una corrida

```
experiments/2026-07-13_strafing_canary/
├── manifest.yaml      # metadatos + configuración
├── notes.md           # observaciones cualitativas
├── bag/               # rosbag2 (NO va a git)
│   ├── metadata.yaml
│   └── *.db3
└── plots/             # PNGs generados (SÍ van a git)
```

## Qué se versiona y qué no

Los bags son binarios grandes y se ignoran vía `.gitignore` (`experiments/*/bag/`). Lo que sí se commitea:

- `manifest.yaml` — para reproducir la corrida sin acceso al bag original.
- `notes.md` — para no perder las observaciones cualitativas.
- `plots/*.png` — porque son los artefactos citables desde el informe.

Si en el futuro un bag chico (< 1 MB) amerita quedar versionado (ej. un canary de regresión), agregá una excepción puntual con `!experiments/<fecha>_<slug>/bag/` al `.gitignore`.

## Ejemplo de manifest — módulo 2 (cuadrado 2m con ganancias default)

```yaml
slug: cuadrado_kpxy08_kpyaw15
descripcion: cuadrado 2m default — baseline del módulo 2

launch:
  paquete: modelo_holonomico
  archivo: odometry.launch.py
  args:
    use_sim_time: true

trayectoria:
  tipo: cuadrado_2m
  parametros:
    lado: 2.0
    kp_xy: 0.8
    kp_yaw: 1.5

cmd_source:
  tipo: closed_loop_launch
  paquete: lazo_cerrado
  archivo: lazo_cerrado.launch.py
  args:
    use_sim_time: true
    square_half_side: 1.0    # el launch usa half_side, no lado
    kp_xy: 0.8
    kp_yaw: 1.5
    goal_mode: PURSUIT
    base_frame: base_link    # cambia a base_link_ekf en módulo 3

duracion_maxima_s: 120

topics_grabados:
  - /robot/odometry
  - /robot/cmd_vel
  - /robot/encoders
  - /tf
  - /tf_static
  - /robot/trajectory        # TransientLocal — referencia, se graba al inicio
  - /goal_pose               # goal activo del pursuit
  - /real_path               # path acumulado
```

Para el **barrido de ganancias** del informe (`kp_xy` × {0.4, 0.8, 1.6} y `kp_yaw` × {0.75, 1.5, 3.0}): copiar el manifest, cambiar dos valores, `run_experiment.sh` de nuevo. Nueve corridas con manifest + plots versionados en git.

## Convenciones de slug

`<trayectoria>_<parámetro-clave>` en snake_case. Ejemplos:

- `strafing_vy02` — strafing puro con `vy=0.2 m/s`
- `circular_vx01_wz01` — círculo analítico `vx=0.1`, `wz=0.1`
- `cuadrado_kpxy08_kpyaw15` — cuadrado 2m con ganancias del controlador
- `ekf_16posts_ruido005` — corrida del módulo 3 con desvío de LiDAR

## Almacenamiento del bag

Por defecto, `record.sh` usa SQLite3 (el default de rosbag2 en Humble) — genera archivos `.db3`. Si querés usar MCAP (más portable, más chico), pasá `--storage mcap` como argumento adicional a `record.sh`; necesitás `ros-humble-rosbag2-storage-mcap` instalado en el contenedor.

## Cuándo saltarse `run_experiment.sh`

`run_experiment.sh` es el camino default, pero podés correr los pasos a mano si necesitás control fino: `bash scripts/record.sh <dir>` en una terminal y publicar `cmd_vel` a mano en otra (útil para debugging o corridas ad-hoc que no encajan en ningún `cmd_source.tipo`). En ese caso, `duracion_maxima_s` es cosmético — no hay watchdog.

## Extensiones previstas

- **Módulo 3 (EKF)**: se agregan `/robot/front_laser/scan`, `/landmarks`, `/posts` a `topics_grabados`; `cmd_source.args.base_frame` pasa a `base_link_ekf`; `plot_run.py` gana `ekf_vs_gt.png` comparando `map → base_link_ekf` vs `map → base_link_gt` (GT sólo para el informe, nunca en el lazo de control).
- **`scripts/sweep.sh`**: barrido de parámetros — dado un manifest base, corre N variantes con overrides. Justificable cuando lleguen las corridas del informe.
- **`plot_run.py --compare <dir1> <dir2> ...`**: overlay de varias corridas para comparar ganancias en un solo plot.
