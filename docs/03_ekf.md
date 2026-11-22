# Módulo 3 — Localización basada en EKF (`robmovil_ekf`)

Paquetes: `robmovil_ekf/` (nodos) y `kfilter/` (librería `EKFilter` de la cátedra, sin modificar).
Base: esqueleto del taller 10 (`robmovil_ekf` del robot diferencial), adaptado al omnidireccional.

## Qué pide la consigna y dónde está

| Ítem | Implementación |
|---|---|
| 1. Nodo LiDAR: postes por clusterización → `/landmarks` (`LandmarkArray`, frame robot) | `landmark_detector_node` (`src/landmark_detector.{h,cpp}`) |
| 2. Modelar x, u, z, w, v, f, h y Jacobianos | `src/localizer_ekf.cpp` (ver §Modelo) |
| 3. Covarianzas: más incertidumbre en la predicción que en el sensado | `makeBaseQ` / `makeBaseR`, σ por parámetro |
| 4.a Mapa de postes en frame `map`, conocido a priori | `on_posts` ← `/posts` |
| 4.b `/robot/odometry` como entrada de control | `on_odometry` → `timeUpdateStep` |
| 4.c Publicar `map → base_link_ekf` | `publish_estimate` (TF + `/robot/odometry_ekf`) |
| 5. Seguir el cuadrado con la pose del EKF | `launch/seguimiento_ekf.launch.py` (controller con `base_frame:=base_link_ekf`) |
| Anexo: `EKFilter(3, 3, 3, 2, 2)` | constructor de `LocalizerEKF` |

## Modelo

| | Dim | Descripción |
|---|---|---|
| x = (x, y, θ) | 3 | pose del robot en `map` |
| u = (vₓ, v_y, ω) | 3 | twist de `/robot/odometry`, en frame del robot |
| z = (r, φ) | 2 | rango y bearing de un poste, en `base_link` |
| w | 3 | ruido de proceso aditivo sobre el estado, N(0, Q) |
| v | 2 | ruido de medición aditivo, N(0, R) |

**Proceso** (las velocidades del cuerpo se rotan por θ para integrar en `map`; Δt = diferencia de stamps de odometría, en sim-time):

```
x' = x + (vₓ cos θ − v_y sin θ) Δt
y' = y + (vₓ sin θ + v_y cos θ) Δt
θ' = θ + ω Δt
```

El módulo 1 publica el twist como Δ/dt con el mismo stamp, así que `u·Δt` reproduce exactamente los incrementos de la odometría.

**Jacobiano A = ∂f/∂x** (evaluado en x_{t−1}; `makeA` corre antes de `makeProcess`):

```
A = | 1  0  −(vₓ sin θ + v_y cos θ) Δt |
    | 0  1   (vₓ cos θ − v_y sin θ) Δt |
    | 0  0   1                         |
```

W = I₃ (ruido aditivo).

**Sensado**, con (lₓ, l_y) el poste del mapa asociado, dx = lₓ − x, dy = l_y − y, q = dx² + dy²:

```
r = √q
φ = atan2(dy, dx) − θ
```

**Jacobiano H = ∂h/∂x:**

```
H = | −dx/√q   −dy/√q    0 |
    |  dy/q    −dx/q    −1 |
```

V = I₂. La innovación del bearing se normaliza a (−π, π] en `makeDZ`.

**Asociación de datos:** cada medición se pasa a `map` con la pose estimada y se asocia al poste más cercano dentro de `association_radius` (0,6 m). Los postes están en una grilla cada 2 m, así que no hay ambigüedad mientras el error de la estimación sea menor a ~1 m. Las mediciones a menos de `min_range` (0,2 m) se descartan (H se vuelve singular). Los updates se aplican secuencialmente, uno por poste.

## Covarianzas (defaults del launcher)

| Parámetro | Valor | Significado |
|---|---|---|
| `sigma_x`, `sigma_y` | 0,15 m | desvío de proceso **por paso de predicción** |
| `sigma_theta` | 0,15 rad | ídem en orientación |
| `sigma_range` | 0,05 m | desvío del rango medido |
| `sigma_bearing` | 0,03 rad | desvío del bearing medido |
| `init_cov` | 0,01 | P₀ = init_cov · I |

Q ≫ R refleja lo que pide la consigna: el filtro confía más en el láser que en la odometría.
**Ojo:** Q es constante por paso, no por segundo; con odometría a ~25 Hz la incertidumbre crece rápido entre correcciones y también con el robot quieto. Candidato a mejora: escalar Q con Δt sobreescribiendo `makeQ()`.

`makeBaseQ`/`makeBaseR` los evalúa la librería recién en el primer paso del filtro, por eso los σ se cargan en el constructor del nodo, antes de cualquier predicción.

## Detector de postes

Puntos válidos del scan → cartesiano en `base_link` (vía TF `base_link → front_laser`) → clusters por distancia entre puntos consecutivos (`cluster_distance_threshold` 0,15 m, entre 2 y 30 puntos) → centroide → (r, φ). Publica también `/landmark_markers` para RViz.
El centroide de los puntos visibles queda algo más cerca del robot que el centro real del poste (radio 5 cm): es un sesgo sistemático chico.

## Integración con el simulador (dos detalles que rompen todo si se ignoran)

1. **`map → odom` lo publica el simulador**, con la pose de spawn `(-2, 2, π/2)` (verificado con `tf2_echo map odom`). Ningún launcher nuestro debe publicarla: con una TF identidad adicional el EKF se inicializa en `(0, 0, 0)` y la asociación de postes falla. El estado inicial se toma de `map → base_link` al llegar la primera odometría.
2. **`/posts` se publica una sola vez, al dar play, con QoS reliable + volatile** (verificado con `ros2 topic info -v /posts`). Una suscripción `transient_local` es incompatible con un publisher volatile y no recibe nada (warning `incompatible QoS ... DURABILITY`). Por eso el nodo se suscribe volatile, y **hay que lanzar los nodos antes de darle play a la simulación**.

## Cómo correrlo

```bash
# con la simulación detenida:
ros2 launch robmovil_ekf seguimiento_ekf.launch.py     # odometría + detector + EKF + controller
# o solo localización (el robot se mueve con otro driver):
ros2 launch modelo_holonomico odometry.launch.py
ros2 launch robmovil_ekf localizer.launch.py [sigma_x:=... only_prediction:=true]
# después: play en CoppeliaSim
```

`only_prediction:=true` desactiva las correcciones (EKF = odometría), útil como control en los experimentos.

## Primera validación (2026-11-22)

Cuadrado completo con `seguimiento_ekf.launch.py` (controller con defaults, lookahead 0,30), comparando contra `base_link_gt` a 1 Hz:

| | error medio | error máx. | error final | yaw final |
|---|---|---|---|---|
| EKF | 0,036 m | 0,049 m | 0,040 m | +0,1° |
| odometría | — | 0,97 m | 0,96 m | −19,3° |

El robot real cerró a ~4 cm de `(-2, 2)`. El error del EKF es casi constante (~4 cm, hacia afuera del cuadrado): probablemente el sesgo del centroide.

## Pendiente

- Experimentos para el informe con el framework (`experiments/`), grabando `/robot/odometry_ekf`: (a) solo localización con velocidad constante, EKF vs odometría vs GT; (b) cuadrado con EKF en el lazo; (c) capturas de RViz con la elipse de covarianza.
- Ajuste de Q (por Δt) y eventual compensación del radio del poste en el detector.
