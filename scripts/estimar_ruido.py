#!/usr/bin/env python3
"""Estima a partir de bags los ruidos del EKF: Q (odometría) y R (postes del láser).

Uso:
    python3 scripts/estimar_ruido.py --out <dir> <bag_dir> [<bag_dir> ...]

Q — error de la odometría en función de la ventana de tiempo τ:
    para cada instante t y cada τ, compara el desplazamiento relativo (en el frame del
    robot en t) según la odometría contra el del ground truth. El RMS de esa diferencia
    es el error que la odometría acumula en τ segundos. Si el error fuese un paseo
    aleatorio (lo que asume Q constante por paso) crecería como √τ; si es sistemático
    (p. ej. slip proporcional al movimiento) crece como τ.

R — error de las detecciones de postes (bags con /landmarks):
    para cada detección se predice (rango, bearing) desde la pose GT contra el poste
    más cercano del mapa, y se reporta media (sesgo) y desvío del residuo.

Correr dentro del container (necesita las libs de ROS 2, igual que plot_run.py).
"""

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

sys.path.insert(0, str(Path(__file__).resolve().parent))
import plot_run as pr  # noqa: E402

DT_ODOM = 0.05  # paso fijo de la odometría en sim-time (medido en los bags)
VENTANAS = [0.05, 0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0]

# Mapa de postes de la escena omni_ekf.ttt (lo que publica /posts, frame map).
POSTES = np.array([
    (0, 4), (-4, 2), (-4, 4), (-4, 0), (0, -4), (2, -4), (-2, 4), (2, 4),
    (4, 4), (4, 2), (4, 0), (4, -2), (4, -4), (-2, -4), (-4, -4), (-4, -2),
], dtype=float)


def cargar(bag):
    m = pr.read_topics(Path(bag), ["/robot/odometry", "/landmarks", "/tf", "/tf_static"])
    off = pr.extract_map_odom_offset(m["/tf"], m["/tf_static"])
    gt = pr.extract_gt_pose(m["/tf"], m["/tf_static"], header_stamp=True)
    odom = pr.odom_sim_time(m["/robot/odometry"], off)
    return odom, gt, m["/landmarks"]


def errores_odometria(odom, gt):
    """Devuelve {τ: (rms_xy, rms_theta)} sobre todas las ventanas posibles."""
    to, xo, yo, tho = odom
    tg, xg, yg, thg = gt
    mask = (to >= tg.min()) & (to <= tg.max())
    to, xo, yo, tho = to[mask], xo[mask], yo[mask], np.unwrap(tho[mask])
    gx, gy, gth = pr.interp_pose(to, tg, xg, yg, thg)
    out = {}
    for tau in VENTANAS:
        k = int(round(tau / DT_ODOM))
        if k >= len(to):
            continue
        i, j = np.arange(len(to) - k), np.arange(k, len(to))

        def rel(x, y, th):
            dx, dy = x[j] - x[i], y[j] - y[i]
            c, s = np.cos(th[i]), np.sin(th[i])
            return c * dx + s * dy, -s * dx + c * dy, th[j] - th[i]

        ox, oy, ot = rel(xo, yo, tho)
        gx_, gy_, gt_ = rel(gx, gy, gth)
        ex, ey, et = gx_ - ox, gy_ - oy, pr.wrap(gt_ - ot)
        rms_xy = math.sqrt(0.5 * (np.mean(ex ** 2) + np.mean(ey ** 2)))
        out[tau] = (rms_xy, math.sqrt(np.mean(et ** 2)))
    return out


def residuos_landmarks(gt, landmarks):
    tg, xg, yg, thg = gt
    dr, db, rr = [], [], []
    for _, msg in landmarks:
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if t < tg.min() or t > tg.max():
            continue
        x, y, th = pr.interp_pose(np.array([t]), tg, xg, yg, thg)
        x, y, th = float(x[0]), float(y[0]), float(th[0])
        for lm in msg.landmarks:
            lx = x + lm.range * math.cos(th + lm.bearing)
            ly = y + lm.range * math.sin(th + lm.bearing)
            d = np.hypot(POSTES[:, 0] - lx, POSTES[:, 1] - ly)
            p = POSTES[np.argmin(d)]
            if d.min() > 0.5:
                continue  # detección espuria (no es un poste del mapa)
            r_pred = math.hypot(p[0] - x, p[1] - y)
            b_pred = math.atan2(p[1] - y, p[0] - x) - th
            dr.append(lm.range - r_pred)
            db.append(math.atan2(math.sin(lm.bearing - b_pred), math.cos(lm.bearing - b_pred)))
            rr.append(r_pred)
    return np.array(dr), np.array(db), np.array(rr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("bags", nargs="+")
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.5))
    todos_dr, todos_db, todos_rr = [], [], []
    for bag in a.bags:
        nombre = Path(bag).parent.name
        odom, gt, lms = cargar(bag)
        e = errores_odometria(odom, gt)
        taus = np.array(sorted(e))
        exy = np.array([e[t][0] for t in taus])
        eth = np.array([e[t][1] for t in taus])
        pend_xy = np.polyfit(np.log(taus), np.log(exy), 1)[0]
        pend_th = np.polyfit(np.log(taus), np.log(eth), 1)[0]
        print(f"\n== {nombre}  (pendiente log-log: xy {pend_xy:.2f}, θ {pend_th:.2f}; "
              f"0.5 = aleatorio, 1 = sistemático)")
        print("   τ [s]   RMS xy [cm]   RMS θ [°]   σ_paso equiv. xy [cm]   θ [°]")
        for t, ex_, et_ in zip(taus, exy, eth):
            esc = math.sqrt(DT_ODOM / t)  # σ_paso que daría ese RMS si fuese aleatorio
            print(f"  {t:5.2f}   {100*ex_:9.2f}   {math.degrees(et_):9.3f}   "
                  f"{100*ex_*esc:12.3f}   {math.degrees(et_)*esc:12.4f}")
        axes[0].loglog(taus, 100 * exy, "o-", label=nombre)
        axes[1].loglog(taus, np.degrees(eth), "o-", label=nombre)
        if lms:
            dr, db, rr = residuos_landmarks(gt, lms)
            todos_dr.append(dr)
            todos_db.append(db)
            todos_rr.append(rr)

    for ax, ref, unidad in ((axes[0], "cm", "xy"), (axes[1], "°", "θ")):
        x = np.array(VENTANAS)
        y0 = ax.get_lines()[0].get_ydata()[0]
        ax.loglog(x, y0 * np.sqrt(x / x[0]), "k:", label="∝ √τ (aleatorio)")
        ax.loglog(x, y0 * (x / x[0]), "k--", label="∝ τ (sistemático)")
        ax.set_xlabel("ventana τ [s]")
        ax.set_ylabel(f"RMS error odometría {unidad} [{ref}]")
        ax.grid(True, which="both", alpha=0.4)
        ax.legend(fontsize=8)
    fig.suptitle("Error de la odometría vs ventana de tiempo (vs ground truth)")
    fig.tight_layout()
    fig.savefig(a.out / "ruido_odometria.png", dpi=120)
    plt.close(fig)

    if todos_dr:
        dr, db, rr = (np.concatenate(v) for v in (todos_dr, todos_db, todos_rr))
        print(f"\n== Detecciones de postes: {len(dr)} asociadas")
        print(f"   rango:   sesgo {100*dr.mean():+.2f} cm   desvío {100*dr.std():.2f} cm")
        print(f"   bearing: sesgo {math.degrees(db.mean()):+.3f}°   desvío {math.degrees(db.std()):.3f}° "
              f"({db.std():.4f} rad)")
        fig, axes = plt.subplots(1, 2, figsize=(11, 4))
        axes[0].scatter(rr, 100 * dr, s=3, alpha=0.4)
        axes[0].axhline(100 * dr.mean(), color="C1", label=f"media {100*dr.mean():+.1f} cm")
        axes[0].set_xlabel("rango real [m]")
        axes[0].set_ylabel("residuo de rango [cm]")
        axes[1].scatter(rr, np.degrees(db), s=3, alpha=0.4)
        axes[1].axhline(math.degrees(db.mean()), color="C1",
                        label=f"media {math.degrees(db.mean()):+.2f}°")
        axes[1].set_xlabel("rango real [m]")
        axes[1].set_ylabel("residuo de bearing [°]")
        for ax in axes:
            ax.grid(True)
            ax.legend()
        fig.suptitle("Residuo de las detecciones de postes (medido − esperado desde el GT)")
        fig.tight_layout()
        fig.savefig(a.out / "ruido_landmarks.png", dpi=120)
        plt.close(fig)


if __name__ == "__main__":
    main()
