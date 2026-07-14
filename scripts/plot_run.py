#!/usr/bin/env python3
"""Genera plots estándar a partir del bag de una corrida experimental.

Uso:
    python3 scripts/plot_run.py <dir-de-corrida>

Lee <dir>/manifest.yaml y <dir>/bag/, y escribe PNGs a <dir>/plots/.
Los plots dependen de `cmd_source.tipo` en el manifest:
  - open_loop_twist  → xy_path, body_velocity, odom_vs_cmd (módulo 1).
  - closed_loop_launch → módulo 1 + tracking_error, yaw_error, goal_pose_vs_pose
                          (módulo 2, requiere /robot/trajectory en el bag).

Requiere: numpy, matplotlib, pyyaml, y las libs de ROS 2 Humble
(rosbag2_py, rclpy.serialization, rosidl_runtime_py), por lo que hay que
ejecutarlo dentro del contenedor tras `source install/setup.bash`.
"""

import argparse
import math
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")  # sin display; sólo escribimos a PNG.
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import yaml  # noqa: E402

try:
    from rclpy.serialization import deserialize_message
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rosidl_runtime_py.utilities import get_message
except ImportError as e:
    print(
        f"Faltan libs de ROS 2 ({e}). "
        "Ejecutá dentro del container tras `source install/setup.bash`.",
        file=sys.stderr,
    )
    sys.exit(1)


REF_TRAJ_TOPIC = "/robot/trajectory"
GOAL_POSE_TOPIC = "/goal_pose"

# Ruedas: convención del proyecto — MultiEncoderTicks.ticks[i] con i en 0..3 =
# [FL, FR, RL, RR]. Los topics de velocidad angular comandada tienen esa nomenclatura.
WHEELS = ["front_left", "front_right", "rear_left", "rear_right"]
WHEEL_CMD_TOPICS = [f"/robot/{w}_wheel/cmd_vel" for w in WHEELS]
ENCODER_TOPIC = "/robot/encoders"
ENCODER_TICKS_PER_REV = 500  # ticks/vuelta — CLAUDE.md, se corresponde con mecanum_odometry.cpp

# Ventana de media móvil (en número de muestras) para el plot per-rueda. Con encoders
# publicando a ~100 Hz, 7 muestras ≈ 70 ms — suficiente para atenuar la cuantización de
# ±1 tick por muestra sin comer transitorios reales del sistema (que están en escala > 1 s).
WHEEL_SMOOTH_WINDOW = 7


def quat_to_yaw(qx: float, qy: float, qz: float, qw: float) -> float:
    """Yaw (Z) desde quaternion — evita traer tf_transformations como dep."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def open_bag(bag_dir: Path):
    """Abre un rosbag2, autodetectando storage por su metadata.yaml."""
    metadata = bag_dir / "metadata.yaml"
    if not metadata.exists():
        raise FileNotFoundError(f"No hay metadata.yaml en {bag_dir}")
    with metadata.open() as f:
        meta = yaml.safe_load(f)
    storage_id = meta["rosbag2_bagfile_information"]["storage_identifier"]

    reader = SequentialReader()
    reader.open(
        StorageOptions(uri=str(bag_dir), storage_id=storage_id),
        ConverterOptions("", ""),
    )
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    return reader, topic_types


def read_topics(bag_dir: Path, topics_of_interest):
    """Lee el bag y devuelve un dict topic -> lista de (t_segundos, mensaje).

    Los topics pedidos que no estén en el bag devuelven lista vacía (no error).
    """
    reader, topic_types = open_bag(bag_dir)
    msgs = {t: [] for t in topics_of_interest}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in msgs:
            continue
        if topic not in topic_types:
            continue
        MsgType = get_message(topic_types[topic])
        msg = deserialize_message(data, MsgType)
        msgs[topic].append((t_ns * 1e-9, msg))
    return msgs


def extract_odom(entries):
    ts, x, y, yaw, vx, vy, wz = [], [], [], [], [], [], []
    for t, m in entries:
        ts.append(t)
        x.append(m.pose.pose.position.x)
        y.append(m.pose.pose.position.y)
        q = m.pose.pose.orientation
        yaw.append(quat_to_yaw(q.x, q.y, q.z, q.w))
        vx.append(m.twist.twist.linear.x)
        vy.append(m.twist.twist.linear.y)
        wz.append(m.twist.twist.angular.z)
    return (np.array(ts), np.array(x), np.array(y), np.array(yaw),
            np.array(vx), np.array(vy), np.array(wz))


def extract_cmd(entries):
    ts, vx, vy, wz = [], [], [], []
    for t, m in entries:
        ts.append(t)
        vx.append(m.linear.x)
        vy.append(m.linear.y)
        wz.append(m.angular.z)
    return np.array(ts), np.array(vx), np.array(vy), np.array(wz)


def extract_trajectory(entries):
    """De una sola robmovil_msgs/Trajectory (QoS TransientLocal) saca (xs, ys, yaws)."""
    if not entries:
        return None
    _, m = entries[-1]  # nos quedamos con la última publicación
    xs, ys, yaws = [], [], []
    for pt in m.points:
        t = pt.transform.translation
        r = pt.transform.rotation
        xs.append(t.x)
        ys.append(t.y)
        yaws.append(quat_to_yaw(r.x, r.y, r.z, r.w))
    return np.array(xs), np.array(ys), np.array(yaws)


def extract_goal_pose(entries):
    """geometry_msgs/PoseStamped → (ts, xs, ys, yaws)."""
    ts, xs, ys, yaws = [], [], [], []
    for t, m in entries:
        ts.append(t)
        p = m.pose.position
        r = m.pose.orientation
        xs.append(p.x)
        ys.append(p.y)
        yaws.append(quat_to_yaw(r.x, r.y, r.z, r.w))
    return np.array(ts), np.array(xs), np.array(ys), np.array(yaws)


def extract_wheel_encoder_velocities(entries):
    """De una secuencia de MultiEncoderTicks calcula ω por rueda por diferencias finitas.

    Devuelve dict {wheel_idx: (ts, omega_rad_s)} para i ∈ 0..3.
    ω_i = (Δticks_i / Δt_sim) × (2π / TICKS_PER_REV).

    IMPORTANTE — dt de sim-time, no de bag time: rosbag2 estampa los mensajes con wall
    clock por default. Si CoppeliaSim corre a simSpeed ≠ 1×, dt_bag y dt_sim divergen y
    ω_medida sale escalada por el real-time factor (ej. simSpeed=0.5× → ω sale mitad).
    Usamos msg.header.stamp (sim-time) para el dt del cociente, y el bag time sólo para
    el eje X del plot — así se alinea con /robot/*_wheel/cmd_vel que no tiene header.
    Coincide con lo que hace mecanum_odometry.cpp:80-81 en C++.
    """
    empty = {i: (np.array([]), np.array([])) for i in range(4)}
    if len(entries) < 2:
        return empty

    ts_bag = np.array([t for t, _ in entries])
    ts_hdr = np.array([
        m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        for _, m in entries
    ])
    ticks_arr = np.array(
        [[m.ticks[i] for i in range(4)] for _, m in entries], dtype=float
    )  # shape (N, 4)

    dt_hdr = np.diff(ts_hdr)              # dt en sim-time — el correcto para el cociente
    dticks = np.diff(ticks_arr, axis=0)   # shape (N-1, 4)
    ts_v = ts_bag[1:]                     # eje X del plot: bag time, para alinear con cmd

    result = {}
    scale = 2.0 * math.pi / ENCODER_TICKS_PER_REV
    for i in range(4):
        with np.errstate(divide="ignore", invalid="ignore"):
            omega = dticks[:, i] * scale / dt_hdr
        omega = np.where(np.isfinite(omega), omega, 0.0)
        result[i] = (ts_v, omega)
    return result


def extract_wheel_cmd(entries):
    """Lista de (t, std_msgs/Float64) → (ts, values). Devuelve arrays vacíos si no hay entries."""
    if not entries:
        return np.array([]), np.array([])
    ts = np.array([t for t, _ in entries])
    values = np.array([m.data for _, m in entries])
    return ts, values


def plot_xy(odom, manifest, out_path: Path, ref_traj=None):
    ts, x, y, yaw, vx, vy, wz = odom
    fig, ax = plt.subplots(figsize=(6, 6))
    ax.plot(x, y, label="odometría")
    if len(x):
        ax.scatter([x[0]], [y[0]], marker="o", label="inicio")
        ax.scatter([x[-1]], [y[-1]], marker="x", label="fin")

    # Todos los puntos que participan en el rango visible (para calcular el bbox cuadrado).
    xs_all = [np.asarray(x)]
    ys_all = [np.asarray(y)]

    # Overlay de la referencia de módulo 2 (si está disponible).
    if ref_traj is not None:
        xr, yr, _ = ref_traj
        if len(xr):
            ax.plot(xr, yr, "--", label="referencia (/robot/trajectory)")
            xs_all.append(np.asarray(xr))
            ys_all.append(np.asarray(yr))
    else:
        # Overlay analítico circular para módulo 1: círculo de radio R = vx/wz,
        # asumiendo el robot arranca en (0, 0) con θ = 0 (giro CCW ⇒ centro en (0, R)).
        tray = manifest.get("trayectoria") or {}
        if tray.get("tipo") == "circular":
            p = tray.get("parametros") or {}
            v = float(p.get("vx", 0.0))
            w = float(p.get("wz", 0.0))
            if abs(w) > 1e-6 and abs(v) > 1e-6:
                R = v / w
                theta = np.linspace(0.0, 2.0 * math.pi, 200)
                x_ana = R * np.sin(theta)
                y_ana = R * (1.0 - np.cos(theta))
                ax.plot(x_ana, y_ana, "--", label=f"circular analítico R={R:.3f} m")
                xs_all.append(x_ana)
                ys_all.append(y_ana)

    # Bbox cuadrado — evita que un canal chico (ej. strafing puro: Δx≈0) se estire
    # visualmente. Tomamos el centro del rango total y el semi-lado como el mayor de
    # (Δx, Δy)/2, con 10% de padding y un mínimo de 0.5 m para casos degenerados.
    x_all = np.concatenate(xs_all)
    y_all = np.concatenate(ys_all)
    half = 0.5
    if len(x_all) and len(y_all):
        xmin, xmax = float(x_all.min()), float(x_all.max())
        ymin, ymax = float(y_all.min()), float(y_all.max())
        xc = 0.5 * (xmin + xmax)
        yc = 0.5 * (ymin + ymax)
        half = 0.5 * max(xmax - xmin, ymax - ymin)
        half = max(half * 1.10, 0.5)
        ax.set_xlim(xc - half, xc + half)
        ax.set_ylim(yc - half, yc + half)

    # Flechas de heading (yaw) muestreadas a lo largo del recorrido — equivalente al
    # arrow de /odometry en RViz. Longitud proporcional al tamaño del plot para que se
    # vean bien tanto en un strafing de 1 m como en un cuadrado de 4 m.
    if len(x) > 1:
        n_arrows = min(15, len(x))
        idx = np.linspace(0, len(x) - 1, n_arrows, dtype=int)
        arrow_len = 0.08 * half  # 8% del semi-lado del plot
        ax.quiver(
            x[idx], y[idx],
            np.cos(yaw[idx]), np.sin(yaw[idx]),
            angles="xy", scale_units="xy", scale=1.0 / arrow_len,
            width=0.008, color="C3", alpha=0.85, label="heading (yaw)",
            zorder=3,
        )

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal")
    ax.grid(True)
    ax.legend()
    ax.set_title(f"Trayectoria (x,y) — {manifest.get('slug', '?')}")
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def plot_body_velocity(odom, cmd, out_path: Path, slug: str):
    ts_o, x, y, yaw, vx_o, vy_o, wz_o = odom
    ts_c, vx_c, vy_c, wz_c = cmd
    if not len(ts_o):
        return
    t0 = ts_o[0]
    fig, axes = plt.subplots(3, 1, figsize=(8, 8), sharex=True)
    canales = [
        (vx_o, vx_c, "v_x", "m/s"),
        (vy_o, vy_c, "v_y", "m/s"),
        (wz_o, wz_c, "ω_z", "rad/s"),
    ]
    for ax, (v_o, v_c, name, unit) in zip(axes, canales):
        ax.plot(ts_o - t0, v_o, label=f"{name} (odom)")
        if len(ts_c):
            ax.plot(ts_c - t0, v_c, "--", label=f"{name} (cmd)")
        ax.set_ylabel(f"{name} [{unit}]")
        ax.grid(True)
        ax.legend()
    axes[-1].set_xlabel("t [s]")
    fig.suptitle(f"Velocidad en cuerpo — {slug}")
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def plot_odom_vs_cmd(odom, cmd, out_path: Path, slug: str):
    """Error ||twist_odom − cmd_vel|| interpolando cmd al reloj de odom."""
    ts_o, x, y, yaw, vx_o, vy_o, wz_o = odom
    ts_c, vx_c, vy_c, wz_c = cmd
    if len(ts_c) < 2 or len(ts_o) < 2:
        return
    t0 = ts_o[0]
    vx_ci = np.interp(ts_o, ts_c, vx_c)
    vy_ci = np.interp(ts_o, ts_c, vy_c)
    wz_ci = np.interp(ts_o, ts_c, wz_c)
    err = np.sqrt((vx_o - vx_ci) ** 2 + (vy_o - vy_ci) ** 2 + (wz_o - wz_ci) ** 2)
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(ts_o - t0, err)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("‖odom.twist − cmd_vel‖")
    ax.grid(True)
    ax.set_title(f"Error de seguimiento de velocidad — {slug}")
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def _closest_indices(x, y, xr, yr):
    """Para cada (x_i, y_i) devuelve el índice del punto más cercano en (xr, yr).

    Broadcast: shape (N, M) donde N=len(x), M=len(xr). Con N=1000 y M=200
    esto son 200k comparaciones — sobra en RAM.
    """
    dx = x[:, None] - xr[None, :]
    dy = y[:, None] - yr[None, :]
    d2 = dx * dx + dy * dy
    return np.argmin(d2, axis=1), np.sqrt(np.min(d2, axis=1))


def plot_tracking_error(odom, ref_traj, out_path: Path, slug: str):
    ts_o, x, y, yaw, *_ = odom
    xr, yr, _ = ref_traj
    if not len(ts_o) or not len(xr):
        return
    _, dist = _closest_indices(x, y, xr, yr)
    t0 = ts_o[0]
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(ts_o - t0, dist)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("distancia al waypoint más cercano [m]")
    ax.grid(True)
    ax.set_title(f"Error de seguimiento (posición) — {slug}")
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def plot_yaw_error(odom, ref_traj, out_path: Path, slug: str):
    ts_o, x, y, yaw, *_ = odom
    xr, yr, yaw_r = ref_traj
    if not len(ts_o) or not len(xr):
        return
    idx, _ = _closest_indices(x, y, xr, yr)
    yaw_ref = yaw_r[idx]
    err = np.array([normalize_angle(a - b) for a, b in zip(yaw_ref, yaw)])
    t0 = ts_o[0]
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(ts_o - t0, err)
    ax.axhline(0.0, color="k", lw=0.5)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("yaw_ref − yaw_odom  [rad]")
    ax.grid(True)
    ax.set_title(f"Error de seguimiento (yaw) — {slug}")
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def plot_goal_pose_vs_pose(odom, goals, out_path: Path, slug: str):
    """Overlay del goal activo vs pose real en x e y en función del tiempo."""
    ts_o, x, y, *_ = odom
    ts_g, xg, yg, _ = goals
    if not len(ts_o) or not len(ts_g):
        return
    t0 = ts_o[0]
    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    axes[0].plot(ts_o - t0, x, label="x (odom)")
    axes[0].plot(ts_g - t0, xg, "--", label="x (goal_pose)")
    axes[0].set_ylabel("x [m]")
    axes[0].legend(); axes[0].grid(True)
    axes[1].plot(ts_o - t0, y, label="y (odom)")
    axes[1].plot(ts_g - t0, yg, "--", label="y (goal_pose)")
    axes[1].set_ylabel("y [m]")
    axes[1].set_xlabel("t [s]")
    axes[1].legend(); axes[1].grid(True)
    fig.suptitle(f"Goal activo vs pose real — {slug}")
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def _moving_average(x, window):
    """Media móvil centrada; los bordes se rellenan con el primer/último valor válido
    para preservar la longitud del array (evita el dip artificial en los extremos que
    aparece cuando el kernel de convolución se sale del rango del signal)."""
    x = np.asarray(x, dtype=float)
    if window <= 1 or len(x) < window:
        return x.copy()
    kernel = np.ones(window) / window
    valid = np.convolve(x, kernel, mode="valid")
    pad_left = (window - 1) // 2
    pad_right = window - 1 - pad_left
    return np.concatenate([
        np.full(pad_left, valid[0]),
        valid,
        np.full(pad_right, valid[-1]),
    ])


def plot_wheel_velocities(enc_wheels, cmd_wheels, out_path: Path, slug: str):
    """2×2 subplots — una rueda por celda. ω_medido (encoders) + ω_comandado (IK).

    enc_wheels: dict {0..3 → (ts, ω)}.
    cmd_wheels: dict {0..3 → (ts, ω)}. Puede tener llaves faltantes o arrays vacíos
                si el bag no incluye los topics /robot/{...}/wheel/cmd_vel (bags viejos).
    """
    if not any(len(v[0]) for v in enc_wheels.values()) and not any(
        len(v[0]) for v in cmd_wheels.values()
    ):
        return

    labels = ["FL (front-left)", "FR (front-right)", "RL (rear-left)", "RR (rear-right)"]

    # t0 común entre todos los canales para que los subplots compartan el eje temporal.
    all_starts = []
    for ts, _ in enc_wheels.values():
        if len(ts):
            all_starts.append(ts[0])
    for ts, _ in cmd_wheels.values():
        if len(ts):
            all_starts.append(ts[0])
    t0 = min(all_starts) if all_starts else 0.0

    fig, axes = plt.subplots(2, 2, figsize=(11, 8), sharex=True)
    axes = axes.flatten()
    for i, ax in enumerate(axes):
        ts_e, omega_e = enc_wheels.get(i, (np.array([]), np.array([])))
        ts_c, omega_c = cmd_wheels.get(i, (np.array([]), np.array([])))
        if len(ts_e):
            if len(omega_e) >= WHEEL_SMOOTH_WINDOW:
                omega_smoothed = _moving_average(omega_e, WHEEL_SMOOTH_WINDOW)
                # Crudo como fondo tenue — deja ver la cuantización sin dominar el plot.
                ax.plot(ts_e - t0, omega_e, color="C0", alpha=0.20, linewidth=0.6)
                ax.plot(ts_e - t0, omega_smoothed, color="C0",
                        label=f"ω medido (media móvil, N={WHEEL_SMOOTH_WINDOW})")
            else:
                ax.plot(ts_e - t0, omega_e, color="C0", label="ω medido (encoder)")
        if len(ts_c):
            ax.plot(ts_c - t0, omega_c, "--", color="C1", label="ω comandado (IK)")
        ax.set_title(labels[i])
        ax.set_ylabel("ω [rad/s]")
        ax.grid(True)
        ax.legend()
    axes[2].set_xlabel("t [s]")
    axes[3].set_xlabel("t [s]")
    fig.suptitle(f"Velocidad angular por rueda — {slug}")
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def is_closed_loop(manifest: dict) -> bool:
    cs = manifest.get("cmd_source") or {}
    if isinstance(cs, str):
        return False  # legacy schema
    return cs.get("tipo") == "closed_loop_launch"


def main():
    parser = argparse.ArgumentParser(
        description="Genera plots estándar de una corrida (bag + manifest)."
    )
    parser.add_argument(
        "run_dir", type=Path,
        help="Directorio de la corrida (experiments/<fecha>_<slug>)",
    )
    args = parser.parse_args()

    run_dir = args.run_dir.resolve()
    manifest_path = run_dir / "manifest.yaml"
    bag_dir = run_dir / "bag"
    plots_dir = run_dir / "plots"

    if not manifest_path.exists():
        print(f"No existe {manifest_path}", file=sys.stderr)
        sys.exit(1)
    if not bag_dir.exists():
        print(f"No existe {bag_dir} — ¿corriste record.sh o run_experiment.sh?", file=sys.stderr)
        sys.exit(1)
    plots_dir.mkdir(exist_ok=True)

    with manifest_path.open() as f:
        manifest = yaml.safe_load(f) or {}

    slug = manifest.get("slug") or run_dir.name
    closed_loop = is_closed_loop(manifest)

    topics = ["/robot/odometry", "/robot/cmd_vel", ENCODER_TOPIC] + WHEEL_CMD_TOPICS
    if closed_loop:
        topics += [REF_TRAJ_TOPIC, GOAL_POSE_TOPIC]

    print(f"Leyendo bag: {bag_dir}  (closed_loop={closed_loop})")
    msgs = read_topics(bag_dir, topics)
    if not msgs["/robot/odometry"]:
        print("El bag no contiene /robot/odometry — abortando.", file=sys.stderr)
        sys.exit(1)

    odom = extract_odom(msgs["/robot/odometry"])
    cmd = extract_cmd(msgs["/robot/cmd_vel"])
    ref_traj = extract_trajectory(msgs.get(REF_TRAJ_TOPIC, [])) if closed_loop else None
    goals = extract_goal_pose(msgs.get(GOAL_POSE_TOPIC, [])) if closed_loop else None

    enc_wheels = extract_wheel_encoder_velocities(msgs.get(ENCODER_TOPIC, []))
    cmd_wheels = {i: extract_wheel_cmd(msgs.get(topic, []))
                  for i, topic in enumerate(WHEEL_CMD_TOPICS)}
    if not any(len(v[0]) for v in cmd_wheels.values()):
        print(
            "Aviso: el bag no contiene /robot/*_wheel/cmd_vel — sólo se plotea ω medido. "
            "Agregá esos topics a topics_grabados para ver la comparación medido vs comandado.",
            file=sys.stderr,
        )

    print("Generando plots...")
    plot_xy(odom, manifest, plots_dir / "xy_path.png", ref_traj=ref_traj)
    plot_body_velocity(odom, cmd, plots_dir / "body_velocity.png", slug)
    plot_odom_vs_cmd(odom, cmd, plots_dir / "odom_vs_cmd.png", slug)
    plot_wheel_velocities(enc_wheels, cmd_wheels, plots_dir / "wheel_velocities.png", slug)

    if closed_loop and ref_traj is not None:
        plot_tracking_error(odom, ref_traj, plots_dir / "tracking_error.png", slug)
        plot_yaw_error(odom, ref_traj, plots_dir / "yaw_error.png", slug)
    elif closed_loop and ref_traj is None:
        print(
            f"Aviso: cmd_source es closed_loop_launch pero el bag no tiene {REF_TRAJ_TOPIC} "
            f"— agregalo a topics_grabados para habilitar tracking_error / yaw_error.",
            file=sys.stderr,
        )

    if closed_loop and goals is not None and len(goals[0]):
        plot_goal_pose_vs_pose(odom, goals, plots_dir / "goal_pose_vs_pose.png", slug)

    print(f"Listo: {plots_dir}")


if __name__ == "__main__":
    main()
