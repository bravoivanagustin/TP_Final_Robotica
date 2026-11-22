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
| 2026-11-22 | Sin barrido de lookahead; tune `kp_yaw = 0.75` | La consigna sólo pide variar las constantes P; se agrega el error de seguimiento vs setpoint |

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

Barridos univariados de las dos constantes del controlador P alrededor de un
punto central (`kp_xy=0.8`, `kp_yaw=1.5`), con 3 valores cada una:

- **`kp_xy`** ∈ {0.4, 0.8, 1.6}
- **`kp_yaw`** ∈ {0.75, 1.5, 3.0}

La distancia de lookahead del pursuit queda fija en 0.30 m en todas las
corridas: es un parámetro de la selección de waypoints, no del controlador, y
la consigna no pide variarlo.

Cada corrida recorre el cuadrado completo (16 m de perímetro) en la escena
`coppeliaSim/omni_ekf.ttt`. Las métricas se calculan en post-proceso a partir
de los bags (`scripts/plot_run.py` para los gráficos).

### Qué se mide

**Seguimiento (lo que pide la consigna):** error entre la pose del robot según
su feedback (odometría) y la referencia, tomando en cada instante el waypoint
más cercano.

- `seg_pos`: distancia media al waypoint más cercano.
- `seg_yaw`: error angular medio (y máximo) contra el yaw de ese waypoint.

**Deslizamiento:** además se reporta el **error GT vs odometría**: la distancia entre la pose real del simulador
(`map → odom → base_link_gt`) y la pose estimada por el módulo 1. Ese error es
el deslizamiento de los rodillos Mecanum, que los encoders no ven, y es lo que
el controlador no puede corregir porque su feedback es la propia odometría.

- `drift_fin`: error GT-odometría al terminar la vuelta.
- `drift_med`: mediana del error a lo largo de la vuelta.
- `drift_max`: error máximo.
- `yaw_err`: error angular GT-odometría al final.

---

## Resultados

| Corrida | `kp_xy` | `kp_yaw` | seg_pos | seg_yaw medio / máx | drift_fin | drift_med | drift_max | yaw_err |
|---|---|---|---|---|---|---|---|---|
| **kp_yaw 0.75** | 0.8 | 0.75 | 2.3 cm | **0.7° / 3.7°** | **0.888 m** | 0.869 m | **0.900 m** | 20.7° |
| kp_xy 0.4 | 0.4 | 1.5 | 2.0 cm | 4.1° / 10.5° | 0.909 m | 0.741 m | 0.923 m | **20.4°** |
| baseline | 0.8 | 1.5 | 2.2 cm | 2.9° / 7.6° | 0.917 m | **0.660 m** | 0.919 m | 20.9° |
| kp_yaw 3.0 | 0.8 | 3.0 | 2.2 cm | 4.0° / 10.6° | 1.011 m | 0.794 m | 1.017 m | 23.0° |
| kp_xy 1.6 | 1.6 | 1.5 | **1.7 cm** | 0.9° / 3.8° | 2.121 m | 2.068 m | 2.193 m | 49.8° |

Todas las corridas completan la trayectoria: el controlador llega al último
waypoint dentro de las tolerancias de posición y yaw. Lo que cambia es dónde
queda físicamente el robot según el GT.

La corrida de `kp_xy = 0.4` tarda más del doble que el baseline.

---

## Análisis

### El seguimiento contra el setpoint es bueno en todos los casos

Según su propio feedback, el robot sigue la referencia con ~2 cm de error medio
en todas las corridas: el controlador P converge con cualquiera de las
ganancias probadas. Las diferencias aparecen en el yaw y, sobre todo, en el
deslizamiento que la odometría no ve.

### La ganancia traslacional es la palanca dominante

- **`kp_xy` alto rompe la trayectoria real.** Con `kp_xy = 1.6` el robot sigue
  muy bien la referencia *según la odometría* (el mejor seg_pos, 1.7 cm), pero
  el drift se duplica (> 2 m) y el error angular real llega a ~50°: a más
  velocidad, más slip de los rodillos. En el `xy_path` el GT recorta las
  esquinas y deforma el cuadrado. Es el ejemplo más claro de que un buen
  seguimiento del setpoint no garantiza una buena trayectoria real cuando el
  feedback es odométrico.
- **`kp_xy` bajo no ayuda.** `kp_xy = 0.4` deja el drift_fin igual, empeora el
  seguimiento angular y tarda más del doble.

### La ganancia angular

- `kp_yaw = 0.75` da el mejor seguimiento angular (0.7° medio, 3.7° máximo) y
  el mejor drift_fin, con el mismo seguimiento en posición. Como contra, el
  drift a mitad de vuelta (drift_med) es mayor que el del baseline.
- `kp_yaw = 3.0` empeora todo: ~10% más de drift y el peor seguimiento
  angular. Con la convención cardinal no hay transiente inicial de
  orientación, así que una ganancia angular agresiva no tiene error grande que
  corregir y solo reacciona a los saltos del cursor del pursuit, sumando
  rotación mientras el robot traslada.

Con una corrida por configuración, diferencias de pocos centímetros en el
drift (0.89 / 0.91 / 0.92 m) están dentro de la variabilidad esperable. Las
conclusiones firmes son las tendencias: ganancias altas → más slip.

---

## Elección del tune final

**`kp_xy = 0.8`, `kp_yaw = 0.75`** (lookahead 0.30 m).

- Mejor seguimiento angular del barrido y mejor drift_fin, sin perder
  seguimiento en posición.
- `kp_xy = 0.8` es el punto medio: subirlo multiplica el slip, bajarlo no
  mejora el drift y duplica el tiempo.
- Quedan como defaults de `lazo_cerrado.launch.py` y `seguimiento_odom.launch.py`.

El drift resultante es ~0.9 m sobre 16 m de recorrido (~5.5%), dentro de lo que
se reporta para plataformas Mecanum con odometría pura (5-15%). Es la cota de
precisión de la odometría por encoders y motiva el módulo 3 (EKF con
landmarks).

---

## Figuras a incluir en el TeX

| # | Ruta | Contenido |
|---|---|---|
| 1 | Grilla con los 5 `experiments/2026-11-22-cuadrado_cardinal_*/plots/xy_path.png` | Efecto de cada ganancia sobre la trayectoria real |
| 2 | `experiments/2026-11-22-cuadrado_cardinal_kpyaw075/plots/goal_pose_vs_pose.png` | Tune elegido: pose vs setpoint en el tiempo |
| 3 | `experiments/2026-11-22-cuadrado_cardinal_kpyaw075/plots/body_velocity.png` | Tune elegido: `vx`, `vy`, `wz` comandados |
| 4 | `experiments/2026-11-22-cuadrado_cardinal_kpyaw075/plots/tracking_error.png` y `yaw_error.png` | Tune elegido: error de seguimiento en el tiempo |
| 5 | `experiments/2026-11-22-cuadrado_cardinal_kpxy16/plots/xy_path.png` | Caso patológico: velocidad alta |

La figura 1 no existe como un único PNG: hay que armarla con `subfigure` en el
TeX o con un script.

Los bags de estas corridas están en el release de GitHub `bags-m2-cardinal`
(ese release también contiene las corridas de lookahead, descartadas del repo).
