#!/usr/bin/env python3
"""
check_jacobians.py -- Verificacion numerica de Jf y Jh del EKF (Mecanum 4 ruedas).

Estado:   x = [px, py, theta]  (frame map)
Control:  u = [dxb, dyb, dth]  (deltas en body frame, de /robot/odometry * dt)
Medicion: z = [range, bearing]  por landmark m = [mx, my] (frame map)

Compara los Jacobianos analiticos contra diferencias finitas centradas.
Ejecutar con: python3 check_jacobians.py

CASO CRITICO: landmark detras del robot con dy=0.0 EXACTO (m=[-2,0]).
  Perturbaciones en py cruzan la discontinuidad atan2(+eps,-2)=+pi y
  atan2(-eps,-2)=-pi, produciendo una diferencia espuria de ~2*pi en la FD
  sin normalizar. Con la normalizacion correcta el error cae a < 1e-6.

Salida esperada: todos PASS, exit code 0.
"""

import sys
import numpy as np

# ---------------------------------------------------------------------------
# Parametros globales
# ---------------------------------------------------------------------------

EPS = 1e-6   # paso de diferencia finita centrada
TOL = 1e-6   # tolerancia de comparacion analitico vs numerico


# ---------------------------------------------------------------------------
# Funciones auxiliares
# ---------------------------------------------------------------------------

def normalize_angle(a: float) -> float:
    """Normaliza un angulo a (-pi, pi]."""
    return np.arctan2(np.sin(a), np.cos(a))


# ---------------------------------------------------------------------------
# Modelo de proceso f(x, u)
# ---------------------------------------------------------------------------

def f(x: np.ndarray, u: np.ndarray) -> np.ndarray:
    """
    Modelo de proceso: f(x, u).

    Integracion de Euler: los deltas del cuerpo (dxb, dyb, dth) se rotan al
    frame map con el angulo previo theta y se suman al estado.

    Equivalente a mecanum_odometry.cpp lineas 108-110:
        x+ = x  + dxb*cos(th) - dyb*sin(th)
        y+ = y  + dxb*sin(th) + dyb*cos(th)
        th+ = normalize_angle(th + dth)
    """
    px, py, th = x
    dxb, dyb, dth = u
    xp  = px + dxb * np.cos(th) - dyb * np.sin(th)
    yp  = py + dxb * np.sin(th) + dyb * np.cos(th)
    thp = normalize_angle(th + dth)
    return np.array([xp, yp, thp])


# ---------------------------------------------------------------------------
# Jacobiano analitico de f: Jf = df/dx  (3x3)
# ---------------------------------------------------------------------------

def Jf_analytic(x: np.ndarray, u: np.ndarray) -> np.ndarray:
    """
    Jacobiano analitico de f respecto a x evaluado en x.

         [ 1   0   -dxb*sin(th) - dyb*cos(th) ]
    Jf = [ 0   1    dxb*cos(th) - dyb*sin(th) ]
         [ 0   0              1                ]

    Jacobianos en deltas (sin dt): las derivadas son respecto a (px, py, th).
    """
    _, _, th = x
    dxb, dyb, _ = u
    J = np.eye(3)
    J[0, 2] = -dxb * np.sin(th) - dyb * np.cos(th)
    J[1, 2] =  dxb * np.cos(th) - dyb * np.sin(th)
    # J[2, 2] = 1  (ya en eye(3))
    return J


# ---------------------------------------------------------------------------
# Modelo de observacion h(x, m)
# ---------------------------------------------------------------------------

def h(x: np.ndarray, m: np.ndarray) -> np.ndarray:
    """
    Modelo de observacion: h(x, m) = [range, bearing].

    Para el poste m=(mx,my) en frame map y pose robot x=(px,py,th):
        dx = mx - px
        dy = my - py
        q  = dx^2 + dy^2
        range   = sqrt(q)
        bearing = normalize_angle(atan2(dy, dx) - th)   [en frame base_link]
    """
    px, py, th = x
    mx, my = m
    dx = mx - px
    dy = my - py
    r  = np.sqrt(dx**2 + dy**2)
    bearing = normalize_angle(np.arctan2(dy, dx) - th)
    return np.array([r, bearing])


# ---------------------------------------------------------------------------
# Jacobiano analitico de h: Jh = dh/dx  (2x3)
# ---------------------------------------------------------------------------

def Jh_analytic(x: np.ndarray, m: np.ndarray) -> np.ndarray:
    """
    Jacobiano analitico de h respecto a x evaluado en (x, m).

    Con dx=mx-px, dy=my-py, r=sqrt(dx^2+dy^2), q=dx^2+dy^2:

           [ -dx/r   -dy/r    0  ]
    Jh  =  [                     ]
           [  dy/q   -dx/q   -1  ]

    Fila 0 -- derivadas de range:
        dh1/dpx = -dx/r,  dh1/dpy = -dy/r,  dh1/dth = 0
    Fila 1 -- derivadas de bearing = atan2(my-py, mx-px) - th:
        dh2/dpx =  dy/q   (regla de la cadena con dx=mx-px, d(dx)/d(px)=-1)
        dh2/dpy = -dx/q
        dh2/dth = -1
    """
    px, py, _ = x
    mx, my = m
    dx = mx - px
    dy = my - py
    r  = np.sqrt(dx**2 + dy**2)
    q  = dx**2 + dy**2

    if r < 1e-9:
        raise ValueError(f"Landmark demasiado cercano al robot: r={r:.2e} (singularidad en Jh)")

    J = np.zeros((2, 3))
    # Fila 0: range
    J[0, 0] = -dx / r
    J[0, 1] = -dy / r
    J[0, 2] =  0.0
    # Fila 1: bearing
    J[1, 0] =  dy / q
    J[1, 1] = -dx / q
    J[1, 2] = -1.0
    return J


# ---------------------------------------------------------------------------
# Diferencias finitas numericas (centradas)
# ---------------------------------------------------------------------------

def Jf_numeric(x: np.ndarray, u: np.ndarray, eps: float = EPS) -> np.ndarray:
    """
    Jacobiano numerico de f respecto a x por diferencias finitas centradas.

    Jf_numeric[i, j] = (f(x + eps*ej, u)[i] - f(x - eps*ej, u)[i]) / (2*eps)

    La componente theta (indice 2) de la diferencia se normaliza para evitar
    saltos de 2*pi cuando theta cruza la discontinuidad de atan2.
    """
    n = len(x)
    J = np.zeros((n, n))
    for j in range(n):
        xp = x.copy(); xp[j] += eps
        xm = x.copy(); xm[j] -= eps
        fp = f(xp, u)
        fm = f(xm, u)
        diff = fp - fm
        # Normalizar componente angular
        diff[2] = normalize_angle(fp[2] - fm[2])
        J[:, j] = diff / (2.0 * eps)
    return J


def Jh_numeric(x: np.ndarray, m: np.ndarray, eps: float = EPS) -> np.ndarray:
    """
    Jacobiano numerico de h respecto a x por diferencias finitas centradas.

    CORRECCIÓN CRITICA para la fila de bearing:
        Jh_numeric[1, j] = arctan2(sin(h_plus[1] - h_minus[1]),
                                   cos(h_plus[1] - h_minus[1])) / (2*eps)

    Sin esta normalizacion, cuando m=[-2, 0] y se perturba py:
      h_plus[1]  = atan2(+eps, -2) - 0  ≈ +pi
      h_minus[1] = atan2(-eps, -2) - 0  ≈ -pi
      diferencia cruda ≈ 2*pi  →  error en FD ≈ 2*pi/eps ≈ 3e6  (FALSO FALLO)
    Con normalizacion la diferencia colapsa a 0 y el error numerico es < 1e-6.
    """
    n = len(x)
    J = np.zeros((2, n))
    for j in range(n):
        xp = x.copy(); xp[j] += eps
        xm = x.copy(); xm[j] -= eps
        h_plus  = h(xp, m)
        h_minus = h(xm, m)
        # Fila 0: range -- lineal, sin necesidad de normalizacion
        J[0, j] = (h_plus[0] - h_minus[0]) / (2.0 * eps)
        # Fila 1: bearing -- NORMALIZAR la diferencia antes de dividir
        diff = np.arctan2(
            np.sin(h_plus[1] - h_minus[1]),
            np.cos(h_plus[1] - h_minus[1])
        )
        J[1, j] = diff / (2.0 * eps)
    return J


# ---------------------------------------------------------------------------
# Casos de prueba
# ---------------------------------------------------------------------------

def make_cases():
    """
    Retorna lista de tuplas (descripcion, x, u, m).

    Cubre casos tipicos y el caso critico con landmark detras del robot
    donde dy=0.0 EXACTO y las perturbaciones en py cruzan la discontinuidad
    de atan2, haciendo que la FD sin normalizar explote.
    """
    cases = [
        # --- Caso general 1 ---
        ("general 1: x=(1,2,0.5), u=(0.1,0.05,0.02), m=(3,4)",
         np.array([1.0, 2.0, 0.5]),
         np.array([0.1, 0.05, 0.02]),
         np.array([3.0, 4.0])),

        # --- Caso general 2: avance puro ---
        ("general 2: avance puro x=(0,0,0), u=(0.2,0,0), m=(2,0)",
         np.array([0.0, 0.0, 0.0]),
         np.array([0.2, 0.0, 0.0]),
         np.array([2.0, 0.0])),

        # --- Strafing puro ---
        ("strafing puro: x=(1,1,0), u=(0,0.1,0), m=(2,2)",
         np.array([1.0, 1.0, 0.0]),
         np.array([0.0, 0.1, 0.0]),
         np.array([2.0, 2.0])),

        # --- Rotacion pura ---
        ("rotacion pura: x=(0,0,1.0), u=(0,0,0.3), m=(1,0)",
         np.array([0.0, 0.0, 1.0]),
         np.array([0.0, 0.0, 0.3]),
         np.array([1.0, 0.0])),

        # --- Landmark a la izquierda ---
        ("landmark a la izquierda: x=(0,0,0), u=(0.05,0,0), m=(0,2)",
         np.array([0.0, 0.0, 0.0]),
         np.array([0.05, 0.0, 0.0]),
         np.array([0.0, 2.0])),

        # --- CASO CRITICO: landmark detras, dy=0.0 EXACTO ---
        # atan2(0+eps,-2)=+pi, atan2(0-eps,-2)=-pi  =>  diferencia cruda ~2*pi
        # sin normalizacion en FD: error_max ~ 2*pi/(2*eps) ~ 3e6
        # con normalizacion en FD: error_max < 1e-6
        ("CRITICO: landmark detras x=(0,0,0), u=(0,0,0), m=(-2,0) [dy=0 exacto]",
         np.array([0.0, 0.0, 0.0]),
         np.array([0.0, 0.0, 0.0]),
         np.array([-2.0, 0.0])),
    ]
    return cases


# ---------------------------------------------------------------------------
# Funciones de verificacion individuales
# ---------------------------------------------------------------------------

def check_Jf(x: np.ndarray, u: np.ndarray, desc: str) -> bool:
    """Verifica Jf analitico vs numerico. Devuelve True si pasa."""
    J_an = Jf_analytic(x, u)
    J_nu = Jf_numeric(x, u)
    err = np.max(np.abs(J_an - J_nu))
    ok = err < TOL
    label = "PASS" if ok else "FAIL"
    print(f"    Jf  [{label}]  error_max = {err:.2e}")
    if not ok:
        print(f"         Analitico:\n{J_an}")
        print(f"         Numerico:\n{J_nu}")
    return ok


def check_Jh(x: np.ndarray, m: np.ndarray, desc: str) -> bool:
    """Verifica Jh analitico vs numerico. Devuelve True si pasa."""
    J_an = Jh_analytic(x, m)
    J_nu = Jh_numeric(x, m)
    err = np.max(np.abs(J_an - J_nu))
    ok = err < TOL
    label = "PASS" if ok else "FAIL"
    print(f"    Jh  [{label}]  error_max = {err:.2e}")
    if not ok:
        print(f"         Analitico:\n{J_an}")
        print(f"         Numerico:\n{J_nu}")
    return ok


# ---------------------------------------------------------------------------
# Demostracion del caso critico (por que es necesaria la normalizacion)
# ---------------------------------------------------------------------------

def demo_critical_case() -> None:
    """
    Muestra explicitamente el error que ocurre SIN normalizar la diferencia
    de bearing en la FD, versus el resultado correcto CON normalizacion.

    m = [-2, 0]: landmark directamente detras del robot.
    Al perturbar py:
      atan2(+eps, -2) ≈ +pi
      atan2(-eps, -2) ≈ -pi
      diferencia cruda = +pi - (-pi) = 2*pi  →  cociente ~ 2*pi / (2*eps) ~ 3e6
    Con arctan2(sin(diff), cos(diff)):
      arctan2(sin(2*pi), cos(2*pi)) = arctan2(0, 1) = 0  →  cociente ~ 0
    """
    print("\n" + "=" * 60)
    print("DEMOSTRACION: caso critico m=[-2, 0] con dy=0.0 exacto")
    print("=" * 60)

    x_test = np.array([0.0, 0.0, 0.0])
    m_test = np.array([-2.0, 0.0])
    J_an   = Jh_analytic(x_test, m_test)

    # FD SIN normalizacion (version incorrecta)
    J_broken = np.zeros((2, 3))
    for j in range(3):
        xp = x_test.copy(); xp[j] += EPS
        xm = x_test.copy(); xm[j] -= EPS
        hp = h(xp, m_test)
        hm = h(xm, m_test)
        J_broken[0, j] = (hp[0] - hm[0]) / (2.0 * EPS)
        J_broken[1, j] = (hp[1] - hm[1]) / (2.0 * EPS)   # SIN normalizar

    # FD CON normalizacion (version correcta)
    J_fixed = Jh_numeric(x_test, m_test)

    err_broken = np.max(np.abs(J_an - J_broken))
    err_fixed  = np.max(np.abs(J_an - J_fixed))

    print(f"\n  Jh analitico:\n{J_an}")
    print(f"\n  Jh FD SIN normalizacion:\n{J_broken}")
    print(f"  Error SIN normalizacion: {err_broken:.3e}  (esperado >> 1)")
    print(f"\n  Jh FD CON normalizacion:\n{J_fixed}")
    print(f"  Error CON normalizacion: {err_fixed:.3e}  (esperado < {TOL:.0e})")

    if err_broken > TOL and err_fixed < TOL:
        print("\n  => Normalizacion NECESARIA y SUFICIENTE: CONFIRMADO.")
    elif err_broken < TOL:
        # Muy raro: dependiendo del eps exacto atan2 podria no saltar
        print("\n  => atan2 no salto para este eps exacto (resultado dependiente del precision de float).")
    else:
        print("\n  => ADVERTENCIA: resultado inesperado. Revisar implementacion.")


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def main() -> int:
    """
    Ejecuta todos los casos de prueba. Devuelve 0 si todos pasan, 1 si alguno falla.
    """
    cases = make_cases()
    all_pass = True

    print("=" * 60)
    print("Verificacion de Jacobianos analiticos del EKF")
    print(f"  EPS (paso FD)  = {EPS:.0e}")
    print(f"  TOL (umbral)   = {TOL:.0e}")
    print("=" * 60)

    for desc, x, u, m in cases:
        print(f"\nCaso: {desc}")
        print(f"  x = {x}")
        print(f"  u = {u}")
        print(f"  m = {m}")
        ok_f = check_Jf(x, u, desc)
        ok_h = check_Jh(x, m, desc)
        case_pass = ok_f and ok_h
        all_pass = all_pass and case_pass
        if not case_pass:
            print("  --> FALLO en este caso")

    print("\n" + "=" * 60)
    if all_pass:
        print("RESULTADO FINAL: TODOS LOS CASOS PASARON [PASS]")
    else:
        print("RESULTADO FINAL: UNO O MAS CASOS FALLARON [FAIL]")
    print("=" * 60)

    # Seccion de demostracion del caso critico
    demo_critical_case()

    return 0 if all_pass else 1


if __name__ == "__main__":
    sys.exit(main())
