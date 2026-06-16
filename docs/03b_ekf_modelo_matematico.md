# Modelado Matemático del EKF — Módulo 3 (Ítems 2 y 3)

## 1. Variables del filtro

| Símbolo | Dimensión | Frame | Fuente de datos |
|---------|-----------|-------|-----------------|
| x = [px, py, θ]ᵀ | 3×1 | map | TF `map → base_link` en t=0 |
| u = [dxb, dyb, dθ]ᵀ | 3×1 | base_link (body) | `/robot/odometry` twist×dt |
| z = [r, φ]ᵀ | 2×1 | base_link | `/landmarks` (LandmarkArray) |
| w = [wₓ, wy, wθ]ᵀ | 3×1 | — | Ruido aditivo de proceso, N(0, Q) |
| v = [vᵣ, vφ]ᵀ | 2×1 | — | Ruido aditivo de medición, N(0, R) |

Dimensiones para el filtro: n=3, nu=3, nw=3, m=2, nv=2.

- `dxb = twist.linear.x * dt` — delta de traslación en x del cuerpo
- `dyb = twist.linear.y * dt` — delta de traslación en y del cuerpo (strafing Mecanum)
- `dθ = twist.angular.z * dt` — delta de rotación
- `dt` se calcula como diferencia de stamps entre mensajes consecutivos de `/robot/odometry`

## 2. Modelo de proceso f(x, u)

Los deltas del cuerpo (dxb, dyb, dθ) se rotan al frame map con el ángulo previo θ (integración de Euler) y se suman al estado. Corresponde exactamente a `mecanum_odometry.cpp` líneas 108-110:

```
px+ = px + dxb·cos(θ) - dyb·sin(θ)
py+ = py + dxb·sin(θ) + dyb·cos(θ)
θ+  = normalize_angle(θ + dθ)
```

Incluyendo ruido aditivo:

```
f(x, u, w) = [px + dxb·cos(θ) - dyb·sin(θ) + wₓ,
              py + dxb·sin(θ) + dyb·cos(θ) + wy,
              normalize_angle(θ + dθ + wθ)]ᵀ
```

El ruido w entra de forma aditiva sobre la salida de f, con la misma dimensión que el estado.

## 3. Jacobiano Jf = ∂f/∂x (3×3)

Derivación de las 9 entradas respecto de (px, py, θ) evaluado en la estimación previa:

**Fila 0 — derivadas de f₁ = px + dxb·cos(θ) - dyb·sin(θ):**
```
∂f₁/∂px = 1
∂f₁/∂py = 0
∂f₁/∂θ  = -dxb·sin(θ) - dyb·cos(θ)
```

**Fila 1 — derivadas de f₂ = py + dxb·sin(θ) + dyb·cos(θ):**
```
∂f₂/∂px = 0
∂f₂/∂py = 1
∂f₂/∂θ  =  dxb·cos(θ) - dyb·sin(θ)
```

**Fila 2 — derivadas de f₃ = θ + dθ:**
```
∂f₃/∂px = 0
∂f₃/∂py = 0
∂f₃/∂θ  = 1
```

Forma matricial:

```
       ⎡ 1   0   -(dxb·sin(θ) + dyb·cos(θ)) ⎤
Jf  =  ⎢ 0   1    (dxb·cos(θ) - dyb·sin(θ)) ⎥
       ⎣ 0   0              1                 ⎦
```

**Jacobiano del ruido de proceso:**

```
Jf_w = ∂f/∂w = I₃
```

El ruido w entra de forma aditiva sobre cada componente del estado, por lo que su Jacobiano es la identidad.

Nota: no aparece ningún factor Δt en Jf porque u = (dxb, dyb, dθ) son deltas directos (no velocidades). La incertidumbre de proceso ya está absorbida en sigma_dx, sigma_dy, sigma_dth.

**Ecuación de propagación de covarianza:**

```
P⁻ = Jf · P · Jfᵀ + Jf_w · Q · Jf_wᵀ = Jf · P · Jfᵀ + Q
```

## 4. Modelo de observación h(x, mᵢ)

Para el landmark i con posición (mx, my) en frame map y pose del robot (px, py, θ):

```
dx = mx - px
dy = my - py
q  = dx² + dy²
r  = sqrt(q)

h(x, mᵢ) = [r,
             normalize_angle(atan2(dy, dx) - θ)]ᵀ
```

El bearing es relativo al robot (el detector entrega observaciones en frame `base_link`), de ahí que se reste θ. La función `normalize_angle` lleva el resultado a (-π, π].

Incluyendo ruido aditivo:

```
h(x, mᵢ, v) = [sqrt(q) + vᵣ,
                normalize_angle(atan2(dy, dx) - θ) + vφ]ᵀ
```

**Nota sobre singularidad r → 0:** cuando el robot está prácticamente encima de un poste, r → 0 y tanto `-dx/r` como `dy/q` divergen. Mitigación: descartar observaciones con `range < min_range` (parámetro ROS 2, default 0.2 m) antes de ejecutar la corrección del EKF.

**Innovación:** el componente de bearing de `z - h(x̄, mᵢ)` debe normalizarse siempre a (-π, π] para evitar saltos de ±2π cuando el landmark está detrás del robot o cuando θ cruza la discontinuidad de atan2.

## 5. Jacobiano Jh = ∂h/∂x (2×3)

Con `r = sqrt(q)` y `q = dx² + dy²`:

**Fila 0 — derivadas de h₁ = sqrt(dx² + dy²):**
```
∂h₁/∂px = ∂sqrt(q)/∂px = (1/2) · 2·dx · (∂dx/∂px) / sqrt(q)
         = (1/2) · 2·dx · (-1) / r = -dx/r

∂h₁/∂py = -dy/r

∂h₁/∂θ  = 0      (r no depende de θ)
```

**Fila 1 — derivadas de h₂ = atan2(dy, dx) - θ:**
```
∂h₂/∂px = ∂atan2(dy, dx)/∂px
         = [dx·(∂dy/∂px) - dy·(∂dx/∂px)] / q
         = [dx·0 - dy·(-1)] / q = dy/q

∂h₂/∂py = [dx·(∂dy/∂py) - dy·(∂dx/∂py)] / q
         = [dx·(-1) - dy·0] / q = -dx/q

∂h₂/∂θ  = -1      (término -θ derivado respecto a θ)
```

Forma matricial:

```
        ⎡ -dx/r    -dy/r     0  ⎤
Jh  =   ⎢                        ⎥
        ⎣  dy/q    -dx/q    -1  ⎦
```

**Jacobiano del ruido de medición:**

```
Jh_v = ∂h/∂v = I₂
```

El ruido v entra de forma aditiva sobre [range, bearing], por lo que su Jacobiano es la identidad 2×2.

## 6. Matrices de covarianza (Ítem 3)

| Parámetro | Sigma | Varianza |
|-----------|-------|----------|
| sigma_dx | 0.15 m/paso | 2.25e-2 m² |
| sigma_dy | 0.15 m/paso | 2.25e-2 m² |
| sigma_dth | 0.15 rad/paso | 2.25e-2 rad² |
| sigma_range | 0.05 m | 2.5e-3 m² |
| sigma_bearing | 0.03 rad | 9e-4 rad² |

**Matriz de covarianza de proceso:**

```
Q = diag(sigma_dx², sigma_dy², sigma_dth²)
  = diag(0.15², 0.15², 0.15²)
  = diag(2.25e-2, 2.25e-2, 2.25e-2)
```

**Matriz de covarianza de medición:**

```
R = diag(sigma_range², sigma_bearing²)
  = diag(0.05², 0.03²)
  = diag(2.5e-3, 9e-4)
```

**Covarianza inicial:**

```
P₀ = diag(init_cov, init_cov, init_cov) = diag(0.01, 0.01, 0.01)
```

Se asume que la pose inicial es conocida (la sim arranca en pose conocida e inicialización via TF `map → base_link`).

**Tabla de ratios Q/R:**

| Componente | Q_ii | R_ii | Ratio |
|----------------|----------|--------|-------|
| posición (x,y) | 2.25e-2 | 2.5e-3 | 9.0x |
| ángulo | 2.25e-2 | 9e-4 | 25.0x |

**Justificación:** Q >> R porque el robot Mecanum tiene deslizamiento lateral importante (alta incertidumbre de proceso) mientras que el LiDAR sobre postes cilíndricos tiene ruido bajo (baja incertidumbre de medición). El filtro confía significativamente más en el sensor que en el modelo cinemático, lo cual es físicamente correcto.

**Por que Q no se escala por dt:** sigma_dx = 0.15 m/paso modela la incertidumbre total del paso de integración. El vector u = (dxb, dyb, dθ) son deltas directos, no velocidades. Si fueran velocidades habría que multiplicar sigma por sqrt(dt), pero al trabajar directamente en deltas la varianza por paso es constante independientemente de dt.

A modo de referencia: a 20 Hz con velocidad nominal de 0.5 m/s, `dxb ≈ 0.025 m`. El desvío sigma_dx = 0.15 m (6x el valor nominal) modela el deslizamiento agresivo de las ruedas Mecanum.

## 7. Parámetros ROS 2

| Parámetro | Tipo | Default | Descripción |
|-----------|------|---------|-------------|
| `sigma_dx` | double | `0.15` | Desvío de ruido en dxb (m por paso de integración) |
| `sigma_dy` | double | `0.15` | Desvío de ruido en dyb (m por paso de integración) |
| `sigma_dth` | double | `0.15` | Desvío de ruido en dθ (rad por paso de integración) |
| `sigma_range` | double | `0.05` | Desvío de medición de range (m) |
| `sigma_bearing` | double | `0.03` | Desvío de medición de bearing (rad) |
| `init_cov` | double | `0.01` | Valor diagonal de P₀ (pose inicial conocida) |

Los nombres de parámetros usan el prefijo `sigma_d*` para reflejar que el control input son deltas (no velocidades). Declarar con `declare_parameter(...)` en el constructor del nodo EKF y usar en la inicialización de Q y P₀.
