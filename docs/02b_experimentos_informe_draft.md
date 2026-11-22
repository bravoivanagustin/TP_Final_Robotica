# Borrador — Subsección "Experimentos" del Módulo 2 (informe TeX)

Draft reusable del contenido para la subsección `\subsection{Experimentos}` de
`\section{Control a lazo cerrado y seguimiento de trayectorias}` del informe
(`informe/Informe TP Final.tex`, TODOs de la subsección Experimentos). Este
archivo **no toca el TeX**: es material para copiar o adaptar al redactar.

Sidecar del deep-dive `docs/02_lazo_cerrado.md`.

## Tabla de cambios de este draft

| Fecha | Cambio | Motivo |
|---|---|---|
| 2026-11-21 | Primera versión, barrido con yaw `atan2(y, x)` | Cerrar módulo 2 |
| 2026-11-22 | Reescrito con el barrido de yaw "cardinal" | Se adoptó la convención de la figura de la consigna; las corridas `atan2` se descartaron |

---

## Convención de orientación

La consigna pide orientación "opuesta al centro". El texto admite dos lecturas:

- **Radial pura**: `yaw = atan2(y, x)` en cada waypoint. En los vértices da
  135°, -135°, -45° y 45° (diagonales).
- **Cardinal**: la que dibuja la figura de la consigna (p. 6). En los vértices
  `(-2,+2)`, `(-2,-2)`, `(+2,-2)`, `(+2,+2)` el robot mira a 90°, 180°, 270° y
  360° (norte, oeste, sur, este), y dentro de cada lado el yaw se interpola
  linealmente, rotando 90° CCW por lado.

Se adoptó la **cardinal** por dos motivos:

1. Es la que muestra la figura de la consigna.
2. El robot spawnea en `(-2, +2)` con yaw 90°, que coincide con el primer
   waypoint cardinal: arranca sin error angular. Con la radial arranca con
   45° de error, y el transiente de corrección mete slip. Con las mismas
   ganancias (`kp_xy=0.8`, `kp_yaw=1.5`, `lookahead=0.30`) la radial dio
   drift_fin = 1.158 m y la cardinal 0.917 m (−21%). Las corridas radiales se
   descartaron del repo (quedan en el historial de git, commit `53e92f0`).

---

## Diseño del barrido

Barridos univariados alrededor de un punto central (`kp_xy=0.8`, `kp_yaw=1.5`,
`lookahead_distance=0.30`), como pide la consigna (al menos 3 valores por
constante):

- **`kp_xy`** ∈ {0.4, 0.8, 1.6}
- **`kp_yaw`** ∈ {0.75, 1.5, 3.0}
- **`lookahead_distance`** ∈ {0.15, 0.30, 0.60}: no es una constante del
  controlador P, pero controla la velocidad comandada por otra vía (el goal
  queda más cerca o más lejos del robot), así que sirve de control.
- **Combinación**: `lookahead=0.15` + `kp_yaw=0.75`, las dos palancas que
  mejoraron el drift_fin respecto del punto central.

Cada corrida recorre el cuadrado completo (16 m de perímetro) en la escena
`coppeliaSim/omni_ekf.ttt`. Las métricas se calculan en post-proceso a partir
de los bags (`scripts/plot_run.py` para los gráficos).

### Qué se mide

Además del error pose-vs-setpoint que pide la consigna, se reporta el
**error GT vs odometría**: la distancia entre la pose real del simulador
(`map → odom → base_link_gt`) y la pose estimada por el módulo 1. Ese error es
el deslizamiento de los rodillos Mecanum, que los encoders no ven, y es lo que
el controlador no puede corregir porque su feedback es la propia odometría.

- `drift_fin`: error GT-odometría al terminar la vuelta.
- `drift_med`: mediana del error a lo largo de la vuelta.
- `drift_max`: error máximo.
- `yaw_err`: error angular GT-odometría al final.

---

## Resultados

| Corrida | `lookahead` | `kp_xy` | `kp_yaw` | drift_fin | drift_med | drift_max | yaw_err |
|---|---|---|---|---|---|---|---|
| **combinación** | 0.15 | 0.8 | 0.75 | **0.796 m** | 0.685 m | **0.811 m** | **18.1°** |
| lookahead 0.15 | 0.15 | 0.8 | 1.5 | 0.835 m | **0.564 m** | 0.840 m | 18.4° |
| kp_yaw 0.75 | 0.30 | 0.8 | 0.75 | 0.888 m | 0.869 m | 0.900 m | 20.7° |
| kp_xy 0.4 | 0.30 | 0.4 | 1.5 | 0.909 m | 0.741 m | 0.923 m | 20.4° |
| **baseline** | 0.30 | 0.8 | 1.5 | 0.917 m | 0.660 m | 0.919 m | 20.9° |
| kp_yaw 3.0 | 0.30 | 0.8 | 3.0 | 1.011 m | 0.794 m | 1.017 m | 23.0° |
| kp_xy 1.6 | 0.30 | 1.6 | 1.5 | 2.121 m | 2.068 m | 2.193 m | 49.8° |
| lookahead 0.60 | 0.60 | 0.8 | 1.5 | 2.184 m | 1.663 m | 2.187 m | 50.1° |

Todas las corridas completan la trayectoria: el controlador llega al último
waypoint dentro de las tolerancias de posición y yaw. Lo que cambia es dónde
queda físicamente el robot según el GT.

Las corridas con lookahead 0.15 tardan aproximadamente el doble que el
baseline, y la de `kp_xy = 0.4` más del doble.

---

## Análisis

### La velocidad traslacional es la palanca dominante

- **Velocidades altas rompen el seguimiento.** `kp_xy = 1.6` y
  `lookahead = 0.60` duplican el drift (> 2 m) y llevan el error angular a
  ~50°. En el `xy_path` el GT recorta las esquinas y deforma el cuadrado.
- **Bajar la velocidad vía lookahead ayuda.** `lookahead = 0.15` es la única
  palanca individual que mejora de forma clara: −9% de drift_fin y −15% de
  drift_med respecto del baseline.
- **Bajar la velocidad vía `kp_xy` no ayuda.** `kp_xy = 0.4` deja el drift_fin
  igual y empeora el drift_med, con más del doble de tiempo de corrida. A
  diferencia del lookahead, `kp_xy` también achica la respuesta ante errores
  laterales, y el robot acumula más desvío dentro de cada lado.

### La ganancia angular importa poco

- `kp_yaw = 0.75` deja el drift_fin parecido al baseline (0.89 contra 0.92 m)
  pero empeora el drift_med: el robot va más atrasado en yaw durante el lado.
- `kp_yaw = 3.0` empeora todo un ~10%. Con la convención cardinal no hay
  transiente inicial de orientación, así que una ganancia angular agresiva no
  tiene error grande que corregir y solo reacciona a los saltos del cursor del
  pursuit, sumando rotación mientras el robot traslada.

### La combinación

`lookahead = 0.15` + `kp_yaw = 0.75` da el mejor drift_fin (0.80 m) y el mejor
error angular, pero peor drift_med que `lookahead = 0.15` solo (0.69 contra
0.56 m). Con una corrida por configuración, la diferencia entre las dos está
dentro de la variabilidad esperable: quedan empatadas.

---

## Elección del tune final

**`lookahead_distance = 0.15`, `kp_xy = 0.8`, `kp_yaw = 1.5`.**

- Junto con la combinación, es el mejor resultado del barrido (drift_fin
  0.84 m, drift_med 0.56 m, el más bajo de todos).
- Cambia un solo parámetro respecto de los defaults, y no depende de la
  mejora marginal de `kp_yaw = 0.75`.
- El costo es el tiempo: la vuelta tarda aproximadamente el doble que con
  `lookahead = 0.30`.

El drift resultante es ~0.8 m sobre 16 m de recorrido (~5%), dentro de lo que
se reporta para plataformas Mecanum con odometría pura (5-15%). Es la cota de
precisión de la odometría por encoders y motiva el módulo 3 (EKF con
landmarks).

---

## Figuras a incluir en el TeX

| # | Ruta | Contenido |
|---|---|---|
| 1 | Grilla con los 8 `experiments/2026-11-22-cuadrado_cardinal_*/plots/xy_path.png` | Efecto de cada palanca sobre la trayectoria real |
| 2 | `experiments/2026-11-22-cuadrado_cardinal_lookahead015/plots/goal_pose_vs_pose.png` | Tune elegido: pose vs setpoint en el tiempo |
| 3 | `experiments/2026-11-22-cuadrado_cardinal_lookahead015/plots/body_velocity.png` | Tune elegido: `vx`, `vy`, `wz` comandados |
| 4 | `experiments/2026-11-22-cuadrado_cardinal_lookahead015/plots/tracking_error.png` | Tune elegido: error de seguimiento en el tiempo |
| 5 | `experiments/2026-11-22-cuadrado_cardinal_kpxy16/plots/xy_path.png` | Caso patológico: velocidad alta |

La figura 1 no existe como un único PNG: hay que armarla con `subfigure` en el
TeX o con un script.

Los bags de estas corridas están en el release de GitHub `bags-m2-cardinal`.
