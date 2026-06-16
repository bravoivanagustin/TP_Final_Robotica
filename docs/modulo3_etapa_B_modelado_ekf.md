# Etapa B — Modelado Matemático del EKF (ítems 2 y 3 de la consigna)

---

## Definición de variables

### Estado

```
x = [x, y, θ]ᵀ    (n = 3)
```

| Variable | Unidad | Frame | Fuente de inicialización |
|---|---|---|---|
| `x` | m | `map` | TF `map → base_link` en t=0 |
| `y` | m | `map` | TF `map → base_link` en t=0 |
| `θ` | rad | `map` | TF `map → base_link` en t=0 |

Nota: el estado EKF se inicializa desde la primera TF `map → base_link` (lookup via `tf_buffer_`), no desde `/robot/odometry.pose.pose` que está en frame `odom`. La sim garantiza `map ≡ odom` en t=0, pero inicializar via TF es la práctica correcta y es independiente de esa coincidencia.

### Control

```
u = [dxb, dyb, dth]ᵀ    (nu = 3)
```

| Variable | Unidad | Frame | Obtención |
|---|---|---|---|
| `dxb` | m | `base_link` (body) | `twist.linear.x * dt` de `/robot/odometry` |
| `dyb` | m | `base_link` (body) | `twist.linear.y * dt` de `/robot/odometry` |
| `dth` | rad | — | `twist.angular.z * dt` de `/robot/odometry` |

`dt` se calcula como diferencia de stamps entre mensajes consecutivos de `/robot/odometry`. Estos deltas son coherentes con la integración de `mecanum_odometry.cpp` (líneas 103-110), donde `dxb`, `dyb`, `dth` son los incrementos directos de la cinemática Mecanum en frame del cuerpo antes de ser rotados al frame `odom`.

### Medición

```
z = [range, bearing]ᵀ    (m = 2)    por landmark detectado
```

| Variable | Unidad | Frame | Fuente |
|---|---|---|---|
| `range` | m | — | Norma euclídea del centroide de cluster, expresado en `base_link` |
| `bearing` | rad | `base_link` | `atan2(cy, cx)` del centroide en `base_link` |

Publicado en `/landmarks` (tipo `robmovil_msgs/LandmarkArray`, frame `base_link`) por `landmark_detector`.

### Ruido de proceso

```
w = [w_dx, w_dy, w_dth]ᵀ    (nw = 3)    w ~ N(0, Q)
```

Ruido aditivo sobre el vector de control integrado; modela deslizamiento de ruedas Mecanum y errores de encoders.

### Ruido de medición

```
v = [v_r, v_β]ᵀ    (nv = 2)    v ~ N(0, R)
```

Ruido aditivo sobre range y bearing; modela cuantización del laser y geometría del cluster.

### Mapa

```
m_j = (mx_j, my_j)    j = 1..16    (frame `map`, fijo)
```

Recibido desde `/posts` (`geometry_msgs/PoseArray`, QoS `transient_local`). No se hardcodea en YAML.

---

## Modelo de proceso f(x, u) y Jacobiano Jf

### Ecuaciones explícitas

Los deltas del cuerpo `(dxb, dyb, dth)` se rotan al frame `map` con el ángulo previo `θ` (integración de Euler) y se suman al estado:

```
x⁺  = x  +  dxb · cos(θ) - dyb · sin(θ)
y⁺  = y  +  dxb · sin(θ) + dyb · cos(θ)
θ⁺  = θ  +  dth                            (normalizar a (-π, π])
```

Incluyendo el ruido aditivo:

```
f(x, u, w) = [x + dxb·cos(θ) - dyb·sin(θ) + w_dx,
              y + dxb·sin(θ) + dyb·cos(θ) + w_dy,
              θ + dth + w_dth]ᵀ
```

### Jacobiano Jf = ∂f/∂x (3×3)

Se deriva respecto de `(x, y, θ)` evaluado en la estimación previa `x̄`:

```
∂f₁/∂x = 1,  ∂f₁/∂y = 0,  ∂f₁/∂θ = -dxb·sin(θ) - dyb·cos(θ)
∂f₂/∂x = 0,  ∂f₂/∂y = 1,  ∂f₂/∂θ =  dxb·cos(θ) - dyb·sin(θ)
∂f₃/∂x = 0,  ∂f₃/∂y = 0,  ∂f₃/∂θ =  1
```

Forma matricial:

```
       ⎡ 1   0   -dxb·sin(θ) - dyb·cos(θ) ⎤
Jf  =  ⎢ 0   1    dxb·cos(θ) - dyb·sin(θ) ⎥
       ⎣ 0   0              1               ⎦
```

Derivación paso a paso del elemento [0,2]:

```
∂/∂θ [dxb·cos(θ) - dyb·sin(θ)] = -dxb·sin(θ) - dyb·cos(θ)
```

Y del elemento [1,2]:

```
∂/∂θ [dxb·sin(θ) + dyb·cos(θ)] = dxb·cos(θ) - dyb·sin(θ)
```

El resto son derivadas triviales (la posición x, y no dependen de x, y directamente, solo de θ).

### Jacobiano Jw = ∂f/∂w (3×3)

El ruido entra de forma aditiva sobre cada componente del estado:

```
       ⎡ 1  0  0 ⎤
Jw  =  ⎢ 0  1  0 ⎥  = I₃
       ⎣ 0  0  1 ⎦
```

### Ecuación de propagación de covarianza

```
P⁻ = Jf · P · Jfᵀ + Jw · Q · Jwᵀ = Jf · P · Jfᵀ + Q
```

---

## Modelo de observación h(x, m_j) y Jacobiano Jh

### Ecuaciones explícitas

Para el poste `j` en `(mx_j, my_j)` (frame `map`) y la pose del robot `(x, y, θ)`:

```
dx = mx_j - x
dy = my_j - y
q  = dx² + dy²

h₁(x) = sqrt(q)               = range esperado
h₂(x) = atan2(dy, dx) - θ     = bearing esperado  (normalizar a (-π, π])
```

El bearing es relativo al robot (el detector lo entrega en frame `base_link`), por eso se resta `θ`.

Incluyendo ruido aditivo:

```
h(x, m_j, v) = [sqrt(q) + v_r,
                atan2(dy, dx) - θ + v_β]ᵀ
```

### Jacobiano Jh = ∂h/∂x (2×3)

Con `r = sqrt(q)`:

Fila 1 — derivadas de `sqrt(q)`:
```
∂h₁/∂x = ∂sqrt(dx² + dy²)/∂x = (1/2) · (2·dx) · (-1) / sqrt(q) = -dx/r
∂h₁/∂y = -dy/r
∂h₁/∂θ = 0
```

Fila 2 — derivadas de `atan2(dy, dx) - θ`:
```
∂h₂/∂x = ∂atan2(dy, dx)/∂x = dy / (dx²+dy²) · (-1) · (-1) 
       → aplicando la regla de la cadena con dx = mx-x:
          ∂(atan2(my-y, mx-x))/∂x = (my-y)·(-1) · (-1) / q  ... 

  Más directamente: atan2(dy,dx) donde dy = my-y, dx = mx-x
  ∂/∂x  atan2(dy, dx) = [dx·(∂dy/∂x) - dy·(∂dx/∂x)] / q
                       = [dx·0 - dy·(-1)] / q = dy/q

∂h₂/∂y = [dx·(∂dy/∂y) - dy·(∂dx/∂y)] / q
        = [dx·(-1) - dy·0] / q = -dx/q

∂h₂/∂θ = -1
```

Forma matricial:

```
        ⎡ -dx/r    -dy/r     0  ⎤
Jh  =   ⎢                        ⎥
        ⎣  dy/q    -dx/q    -1  ⎦
```

### Jacobiano Jv = ∂h/∂v (2×2)

El ruido entra de forma aditiva:

```
       ⎡ 1  0 ⎤
Jv  =  ⎢      ⎥  = I₂
       ⎣ 0  1 ⎦
```

### Innovación (residuo)

```
y_innov = z - h(x̄, m_j)
```

El componente de bearing `y_innov[1]` debe normalizarse siempre a `(-π, π]` para evitar saltos de ±2π cuando el landmark está detrás del robot o cuando `θ` cruza la discontinuidad de atan2. Sin esta normalización, un residuo de ~2π destruye la corrección.

---

## Matrices de covarianza Q, R, P₀

### Parámetros ROS 2 corregidos

Los nombres de parámetros reflejan el modelo de deltas, no velocidades:

| Parámetro | Tipo | Default corregido | Descripción |
|---|---|---|---|
| `sigma_dx` | double | `0.15` | Desvío de ruido en dxb (m por paso) |
| `sigma_dy` | double | `0.15` | Desvío de ruido en dyb (m por paso) |
| `sigma_dth` | double | `0.15` | Desvío de ruido en dth (rad por paso) |
| `sigma_range` | double | `0.05` | Desvío de medición de range (m) |
| `sigma_bearing` | double | `0.03` | Desvío de medición de bearing (rad) |
| `init_cov` | double | `0.01` | Diagonal inicial de P₀ |

### Matriz Q (covarianza de proceso)

```
Q = diag(sigma_dx², sigma_dy², sigma_dth²)
  = diag(0.15², 0.15², 0.15²)
  = diag(2.25e-2, 2.25e-2, 2.25e-2)
```

Q es fija por paso de integración (no se escala por `dt` cuando el control es en deltas, no velocidades).

### Matriz R (covarianza de medición)

```
R = diag(sigma_range², sigma_bearing²)
  = diag(0.05², 0.03²)
  = diag(2.5e-3, 9.0e-4)
```

### Tabla de ratios Q/R (Q >> R exigido)

| Componente | sigma | varianza | Q_ii / R_ii |
|---|---|---|---|
| dx (proceso) | 0.15 m | 2.25e-2 | Q[0,0] / R[0,0] = 2.25e-2 / 2.5e-3 = **9.0x** |
| dy (proceso) | 0.15 m | 2.25e-2 | Q[1,1] / R[0,0] = 2.25e-2 / 2.5e-3 = **9.0x** |
| dth (proceso) | 0.15 rad | 2.25e-2 | Q[2,2] / R[1,1] = 2.25e-2 / 9.0e-4 = **25.0x** |
| range (sensor) | 0.05 m | 2.5e-3 | — |
| bearing (sensor) | 0.03 rad | 9.0e-4 | — |

La relación Q >> R se cumple con ratio real de **9x** para la comparación range y **25x** para bearing, lo que significa que el filtro confía significativamente más en el sensor que en el modelo cinemático. Esto es físicamente correcto: las ruedas Mecanum tienen deslizamiento importante, mientras que el laser tiene ruido bajo.

Justificación de sigma_dx = 0.15 m: un paso típico de integración a 20 Hz corresponde a `dt ≈ 0.05 s`; a velocidad nominal de 0.5 m/s, `dxb ≈ 0.025 m`. Un desvío de 0.15 m por paso (6x el valor nominal) modela el deslizamiento agresivo de las ruedas Mecanum. La relación Q/R anterior con sigma_dx = 0.05 m daba ratio ~1x, haciendo al filtro casi indiferente entre proceso y sensor.

### Matriz P₀ (covarianza inicial)

```
P₀ = diag(init_cov, init_cov, init_cov) = diag(0.01, 0.01, 0.01)
```

Se asume que la pose inicial es conocida (la sim arranca en pose conocida). Si la pose inicial es incierta, subir a `diag(1.0, 1.0, 1.0)`.

---

## Paso 5 — Script check_jacobians.py (versión corregida)

El script verifica numéricamente que los Jacobianos analíticos `Jf` y `Jh` coinciden con diferencias finitas centradas. La corrección clave respecto a la versión original es la normalización del bearing en las diferencias finitas de `h`: sin ella, cuando un landmark queda detrás del robot (`bearing ≈ ±π`), la diferencia `h_plus[1] - h_minus[1]` explota en la discontinuidad de atan2 y el test reporta un falso fallo.

```python
#!/usr/bin/env python3
"""
check_jacobians.py — Verificación numérica de Jf y Jh del EKF.

Compara los Jacobianos analíticos contra diferencias finitas centradas.
Ejecutar con: python3 check_jacobians.py

Casos de prueba:
  - pose general
  - landmark al frente (bearing ≈ 0)
  - landmark a la izquierda (bearing ≈ π/2)
  - landmark DETRÁS del robot (bearing ≈ ±π)  ← caso que requiere normalización
  - landmark en eje x negativo (bearing ≈ π)

Salida esperada: todos los errores < 1e-6 (tolerancia de diferencias finitas).
"""

import numpy as np

EPS = 1e-6       # paso de diferencia finita
TOL = 1e-5       # tolerancia de comparación (>EPS para evitar ruido)


# ---------------------------------------------------------------------------
# Funciones del modelo
# ---------------------------------------------------------------------------

def normalize_angle(a: float) -> float:
    """Normaliza un ángulo a (-π, π]."""
    return np.arctan2(np.sin(a), np.cos(a))


def f(x: np.ndarray, u: np.ndarray) -> np.ndarray:
    """
    Modelo de proceso: f(x, u) donde x=[x,y,th], u=[dxb,dyb,dth].
    Integración de Euler con deltas del cuerpo rotados al frame map.
    """
    px, py, th = x
    dxb, dyb, dth = u
    xp = px + dxb * np.cos(th) - dyb * np.sin(th)
    yp = py + dxb * np.sin(th) + dyb * np.cos(th)
    thp = normalize_angle(th + dth)
    return np.array([xp, yp, thp])


def Jf_analytic(x: np.ndarray, u: np.ndarray) -> np.ndarray:
    """
    Jacobiano analítico de f respecto a x: df/dx (3x3).
    Evaluado en la estimación previa x.
    """
    _, _, th = x
    dxb, dyb, _ = u
    J = np.eye(3)
    J[0, 2] = -dxb * np.sin(th) - dyb * np.cos(th)
    J[1, 2] =  dxb * np.cos(th) - dyb * np.sin(th)
    return J


def h(x: np.ndarray, m: np.ndarray) -> np.ndarray:
    """
    Modelo de observación: h(x, m) donde x=[x,y,th], m=[mx,my].
    Devuelve [range, bearing] del landmark m visto desde la pose x.
    El bearing se normaliza a (-π, π].
    """
    px, py, th = x
    mx, my = m
    dx = mx - px
    dy = my - py
    r = np.sqrt(dx**2 + dy**2)
    bearing = normalize_angle(np.arctan2(dy, dx) - th)
    return np.array([r, bearing])


def Jh_analytic(x: np.ndarray, m: np.ndarray) -> np.ndarray:
    """
    Jacobiano analítico de h respecto a x: dh/dx (2x3).
    Evaluado en la pose x y el landmark m.
    """
    px, py, _ = x
    mx, my = m
    dx = mx - px
    dy = my - py
    r = np.sqrt(dx**2 + dy**2)
    q = dx**2 + dy**2

    if r < 1e-9:
        raise ValueError("Landmark demasiado cercano al robot: r ≈ 0")

    J = np.zeros((2, 3))
    # Fila 0: derivadas de range
    J[0, 0] = -dx / r
    J[0, 1] = -dy / r
    J[0, 2] =  0.0
    # Fila 1: derivadas de bearing = atan2(my-y, mx-x) - θ
    J[1, 0] =  dy / q
    J[1, 1] = -dx / q
    J[1, 2] = -1.0
    return J


# ---------------------------------------------------------------------------
# Verificación por diferencias finitas
# ---------------------------------------------------------------------------

def Jf_numeric(x: np.ndarray, u: np.ndarray, eps: float = EPS) -> np.ndarray:
    """Jacobiano numérico de f respecto a x por diferencias finitas centradas."""
    n = len(x)
    J = np.zeros((n, n))
    for i in range(n):
        xp = x.copy(); xp[i] += eps
        xm = x.copy(); xm[i] -= eps
        fp = f(xp, u)
        fm = f(xm, u)
        # Normalizar diferencia en componente angular (θ en índice 2)
        diff = fp - fm
        diff[2] = normalize_angle(fp[2] - fm[2])  # evitar salto ±2π en θ
        J[:, i] = diff / (2.0 * eps)
    return J


def Jh_numeric(x: np.ndarray, m: np.ndarray, eps: float = EPS) -> np.ndarray:
    """
    Jacobiano numérico de h respecto a x por diferencias finitas centradas.

    CORRECCIÓN CRÍTICA: la diferencia de bearing entre h_plus y h_minus debe
    normalizarse a (-π, π] antes de dividir por 2*eps. Sin esto, cuando el
    landmark está detrás del robot (bearing ≈ ±π), atan2 puede saltar de
    +π a -π entre h_plus y h_minus, produciendo una diferencia espuria de
    ~±2π y un falso fallo de validación.

    Forma correcta:
        diff_bearing = arctan2(sin(h_plus[1] - h_minus[1]),
                               cos(h_plus[1] - h_minus[1])) / (2*eps)

    Forma incorrecta (NO usar):
        diff_bearing = (h_plus[1] - h_minus[1]) / (2*eps)
    """
    n = len(x)
    m_obs = 2  # dimensión de z: [range, bearing]
    J = np.zeros((m_obs, n))
    for i in range(n):
        xp = x.copy(); xp[i] += eps
        xm = x.copy(); xm[i] -= eps
        hp = h(xp, m)
        hm = h(xm, m)
        # Diferencia de range: lineal, no necesita normalización
        J[0, i] = (hp[0] - hm[0]) / (2.0 * eps)
        # Diferencia de bearing: NORMALIZAR para evitar saltos en discontinuidad
        # de atan2 cuando bearing ≈ ±π
        bearing_diff = np.arctan2(
            np.sin(hp[1] - hm[1]),
            np.cos(hp[1] - hm[1])
        ) / (2.0 * eps)
        J[1, i] = bearing_diff
    return J


# ---------------------------------------------------------------------------
# Casos de prueba
# ---------------------------------------------------------------------------

def make_cases():
    """
    Devuelve lista de (nombre, x, u, m) para probar.
    Incluye el caso landmark detrás del robot, que es el que falla
    sin la normalización en las FD.
    """
    # Estado base: robot en (1, 2, π/4)
    x0 = np.array([1.0, 2.0, np.pi / 4])
    u0 = np.array([0.05, 0.02, 0.01])  # deltas típicos a 20 Hz

    cases = [
        # (nombre, x, u, landmark_m)
        ("general",
         x0,
         u0,
         np.array([4.0, 5.0])),

        ("landmark al frente (bearing≈0)",
         np.array([0.0, 0.0, 0.0]),
         u0,
         np.array([3.0, 0.0])),

        ("landmark a la izquierda (bearing≈π/2)",
         np.array([0.0, 0.0, 0.0]),
         u0,
         np.array([0.0, 2.0])),

        ("landmark detrás — bearing≈±π (caso crítico para FD)",
         np.array([0.0, 0.0, 0.0]),
         u0,
         # dy=0.0 exacto: perturbaciones en y cruzan la discontinuidad atan2(0±eps, -2) = ±π
         np.array([-2.0, 0.0])),

        ("pose con θ grande (≈3π/4)",
         np.array([1.0, 1.0, 3 * np.pi / 4]),
         np.array([0.03, -0.01, -0.02]),
         np.array([0.5, 3.0])),

        ("landmark en eje x negativo con θ=π",
         np.array([2.0, 0.0, np.pi]),
         u0,
         np.array([-1.0, 0.0])),
    ]
    return cases


# ---------------------------------------------------------------------------
# Runner principal
# ---------------------------------------------------------------------------

def check_Jf(x: np.ndarray, u: np.ndarray, name: str) -> bool:
    """Verifica Jf analítico vs numérico. Devuelve True si pasa."""
    J_an = Jf_analytic(x, u)
    J_nu = Jf_numeric(x, u)
    err = np.max(np.abs(J_an - J_nu))
    status = "OK" if err < TOL else "FALLO"
    print(f"  Jf [{name}]:  error_max = {err:.2e}  [{status}]")
    return err < TOL


def check_Jh(x: np.ndarray, m: np.ndarray, name: str) -> bool:
    """Verifica Jh analítico vs numérico. Devuelve True si pasa."""
    J_an = Jh_analytic(x, m)
    J_nu = Jh_numeric(x, m)
    err = np.max(np.abs(J_an - J_nu))
    status = "OK" if err < TOL else "FALLO"
    print(f"  Jh [{name}]:  error_max = {err:.2e}  [{status}]")
    return err < TOL


def main():
    cases = make_cases()
    all_pass = True

    print("=" * 60)
    print("Verificación de Jacobianos analíticos del EKF")
    print(f"  eps (FD) = {EPS:.0e},  tolerancia = {TOL:.0e}")
    print("=" * 60)

    for name, x, u, m in cases:
        print(f"\nCaso: {name}")
        print(f"  x = {x},  u = {u}")
        print(f"  m = {m}")
        ok_f = check_Jf(x, u, name)
        ok_h = check_Jh(x, m, name)
        all_pass = all_pass and ok_f and ok_h

    print("\n" + "=" * 60)
    if all_pass:
        print("RESULTADO: TODOS LOS CASOS PASARON")
    else:
        print("RESULTADO: ALGÚN CASO FALLÓ — revisar Jacobiano analítico")
    print("=" * 60)

    # Demostración explícita del fallo SIN normalización en FD de bearing
    print("\n--- Demostración: fallo sin normalización en FD de bearing ---")
    x_test = np.array([0.0, 0.0, 0.0])
    # dy=0.0 exacto: perturbaciones en y cruzan la discontinuidad atan2(0±eps, -2) = ±π
    m_test = np.array([-2.0, 0.0])    # landmark detrás
    u_test = np.array([0.05, 0.02, 0.01])

    # FD sin normalización (versión incorrecta)
    J_broken = np.zeros((2, 3))
    for i in range(3):
        xp = x_test.copy(); xp[i] += EPS
        xm = x_test.copy(); xm[i] -= EPS
        hp = h(xp, m_test)
        hm = h(xm, m_test)
        J_broken[0, i] = (hp[0] - hm[0]) / (2.0 * EPS)
        J_broken[1, i] = (hp[1] - hm[1]) / (2.0 * EPS)   # SIN normalizar

    J_an = Jh_analytic(x_test, m_test)
    err_broken = np.max(np.abs(J_an - J_broken))
    err_fixed  = np.max(np.abs(J_an - Jh_numeric(x_test, m_test)))

    print(f"  Error SIN normalización en FD: {err_broken:.2e}  (puede ser >>1)")
    print(f"  Error CON normalización en FD: {err_fixed:.2e}   (debe ser <{TOL:.0e})")
    if err_broken > TOL and err_fixed < TOL:
        print("  => Normalización NECESARIA y SUFICIENTE: confirmado.")
    elif err_broken < TOL:
        print("  => atan2 no saltó en este caso específico (depende del eps exacto).")


if __name__ == "__main__":
    main()
```

Guardar en el paquete como `ekf_localizer/scripts/check_jacobians.py` y ejecutar con:

```bash
python3 check_jacobians.py
```

Salida esperada (todos OK):

```
============================================================
Verificación de Jacobianos analíticos del EKF
  eps (FD) = 1e-06,  tolerancia = 1e-05
============================================================

Caso: general
  Jf [general]:  error_max = X.XXe-0Y  [OK]
  Jh [general]:  error_max = X.XXe-0Y  [OK]
...
Caso: landmark detrás — bearing≈π (caso crítico para FD)
  Jf [...]:  error_max = X.XXe-0Y  [OK]
  Jh [...]:  error_max = X.XXe-0Y  [OK]   ← fallaría sin normalización

RESULTADO: TODOS LOS CASOS PASARON
```

---

## Riesgos

**Falso fallo en check_jacobians.py para landmark detrás (φ ≈ ±π).**
La diferencia `h_plus[1] - h_minus[1]` puede valer ~±2π cuando atan2 salta en su discontinuidad al perturbar ligeramente el estado. La corrección es usar `arctan2(sin(diff), cos(diff)) / (2*eps)` en lugar de `diff / (2*eps)`. El script ya incluye esta corrección; documentado en la función `Jh_numeric`.

**Q no cumple Q >> R con sigma_dx = sigma_range = 0.05 m.**
El ratio Q[0,0]/R[0,0] con esos valores es ~1x (indiferencia entre proceso y sensor). Con los valores corregidos `sigma_dx = sigma_dy = sigma_dth = 0.15` el ratio es 9x (range) y 25x (bearing), garantizando que el filtro confía en el sensor. Ver tabla de ratios en la sección anterior.

**Inicialización de estado desde frame equivocado.**
Si se inicializa desde `/robot/odometry.pose.pose` se obtiene la pose en frame `odom`, no en `map`. Si `map ≡ odom` al arrancar se ve bien, pero si no coinciden (ej. relanzan la sim sin reiniciar el EKF), el estado arranca en la posición incorrecta. La práctica correcta es lookupTransform `map → base_link` para la inicialización.

**Singularidad en Jh con r ≈ 0.**
Si el robot está prácticamente encima del poste, `r → 0` y `Jh` diverge (división por cero en `-dx/r` y `dy/q`). Mitigación: descartar observaciones con `range < min_range` (parámetro ROS 2, default 0.2 m) antes de ejecutar la corrección.

**Desvío sigma_dth en unidades inconsistentes.**
`sigma_dth = 0.15` rad por paso de integración. A 20 Hz y `wz = 0.3 rad/s`, el delta nominal es `dth ≈ 0.015 rad`. El desvío modelado (0.15 rad) es 10x el valor nominal — apropiado para un robot con ruedas Mecanum de alta incertidumbre rotacional. Verificar que la unidad en el parámetro ROS 2 sea rad (no rad/s).

**Nombres de parámetros inconsistentes con el plan base.**
El plan base `03_ekf_localizer_plan.md` usa `sigma_v`, `sigma_vy`, `sigma_w` (nombres de velocidades). Este documento corrige los nombres a `sigma_dx`, `sigma_dy`, `sigma_dth` para reflejar que el control input son deltas, no velocidades. Actualizar el CMakeLists y el código C++ en consecuencia al declarar los parámetros con `declare_parameter(...)`.
