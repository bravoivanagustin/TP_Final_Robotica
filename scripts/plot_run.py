#!/usr/bin/env python3
"""Genera plots estándar a partir del bag de una corrida experimental.

Uso:
    python3 scripts/plot_run.py <dir-de-corrida>

Lee <dir>/manifest.yaml y <dir>/bag/, y escribe PNGs a <dir>/plots/.
Los plots dependen de `cmd_source.tipo` en el manifest:
  - open_loop_twist  → xy_path, body_velocity, odom_vs_cmd, wheel_velocities (módulo 1).
  - closed_loop_launch → módulo 1 + tracking_error, yaw_error, goal_pose_vs_pose
                          (módulo 2, requiere /robot/trajectory en el bag).
  - si el bag tiene /robot/odometry_ekf (módulo 3): ekf_vs_odom_error, ekf_error_2sigma,
    ekf_sigma, ekf_nees y metricas_ekf.yaml (errores, % dentro de ±2σ, NEES).

xy_path dibuja la pose integrada por odometría. Si el bag contiene /tf con un
TF `child=base_link_gt` (sim lo publica si grabamos /tf y /tf_static), suma como
overlay la trayectoria ground truth reconstruida en frame `map` — la brecha
odometría↔GT es exactamente el drift que motiva el EKF del módulo 3.

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


EKF_TOPIC = "/robot/odometry_ekf"
LANDMARKS_TOPIC = "/landmarks"
# Cuantiles 2.5% y 97.5% de una chi-cuadrado con 3 gl (estado x, y, θ). Un filtro
# consistente tiene NEES dentro de este intervalo ~95% del tiempo, y media ≈ 3.
NEES_CHI2_3_LO, NEES_CHI2_3_HI = 0.216, 9.348

# Postes de la escena omni_ekf.ttt (lo que publica /posts una vez al dar play, en frame
# map). Se fijan acá porque el grabador suele perderse ese único mensaje.
POSTES_XY = np.array([
    (0, 4), (-4, 2), (-4, 4), (-4, 0), (0, -4), (2, -4), (-2, 4), (2, 4),
    (4, 4), (4, 2), (4, 0), (4, -2), (4, -4), (-2, -4), (-4, -4), (-4, -2),
], dtype=float)

REF_TRAJ_TOPIC = "/robot/trajectory"
GOAL_POSE_TOPIC = "/goal_pose"
TF_TOPICS = ("/tf", "/tf_static")

# Frame del ground truth que publica CoppeliaSim. CLAUDE.md §"Simulator interface" lo
# establece: el sim mantiene map → odom (pose de spawn) y odom → base_link_gt como cadena
# paralela a la que nuestra odometría integra (odom → base_link). Lo usamos SOLO para
# evidencia (plots, métricas) — nunca como feedback de control (ídem consigna).
GT_CHILD_FRAME = "base_link_gt"

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


def extract_gt_pose(tf_entries, tf_static_entries, header_stamp=False):
    """Reconstruye la pose GT en frame map a partir de /tf y /tf_static.

    Devuelve (ts, xs, ys, yaws) con la pose de `base_link_gt` expresada en `map`.
    ts es bag-time por default; con header_stamp=True es el stamp de la TF (sim-time),
    que es lo que hace falta para alinear con /robot/odometry_ekf muestra a muestra.

    Convención de la escena (ver CLAUDE.md §"Simulator interface"):
      map → odom          (publicada por el sim; estática o ~6 Hz dinámica)
      odom → base_link_gt (publicada ~10 Hz por el sim)

    Componemos la cadena combinando /tf (dinámico) y /tf_static. Para cada
    arista mantenemos la "última vista": esto maneja tanto el caso estático
    puro como el caso en el que map→odom venga por /tf como en nuestro sim.

    Si no aparece ningún TF con child == base_link_gt, devolvemos arrays vacíos y
    el caller decide si avisar al usuario.
    """
    # Snapshot de la última TF vista para cada arista. Las estáticas se cargan
    # primero; las dinámicas las van pisando si llegan (lo deseado).
    latest = {}  # (parent, child) -> TransformStamped
    for _, msg in tf_static_entries:
        for tr in msg.transforms:
            latest[(tr.header.frame_id, tr.child_frame_id)] = tr

    def compose_to_map(parent, tx, ty, yaw):
        """Si parent != map, camina hacia atrás por el `latest` hasta llegar a `map`.
        Devuelve (x_map, y_map, yaw_map) o None si la cadena no se cierra."""
        x, y, th = tx, ty, yaw
        current = parent
        for _ in range(6):  # tope de seguridad por si hay ciclo o cadena muy larga
            if current == "map":
                return x, y, th
            # Buscamos la última TF <algo> → current conocida.
            found = None
            for (p, c), tr in latest.items():
                if c == current:
                    found = (p, tr)
                    break
            if found is None:
                return None
            p, tr = found
            # pose_in_p = T_p_current · pose_in_current
            c, s = math.cos(_tf_yaw(tr)), math.sin(_tf_yaw(tr))
            x_new = tr.transform.translation.x + c * x - s * y
            y_new = tr.transform.translation.y + s * x + c * y
            th += _tf_yaw(tr)
            x, y = x_new, y_new
            current = p
        return None

    ts, xs, ys, yaws = [], [], [], []
    for t_bag, msg in tf_entries:
        # Primero pisamos el snapshot con todo lo que trae este TFMessage.
        # Hacemos esto ANTES de procesar las entradas de GT en este mismo
        # mensaje: si el sim publica map→odom en el mismo tick que odom→GT,
        # queremos usar el map→odom actualizado.
        for tr in msg.transforms:
            latest[(tr.header.frame_id, tr.child_frame_id)] = tr
        for tr in msg.transforms:
            if tr.child_frame_id != GT_CHILD_FRAME:
                continue
            tx = tr.transform.translation.x
            ty = tr.transform.translation.y
            r = tr.transform.rotation
            yaw = quat_to_yaw(r.x, r.y, r.z, r.w)
            parent = tr.header.frame_id
            if parent == "map":
                x_m, y_m, yaw_m = tx, ty, yaw
            else:
                composed = compose_to_map(parent, tx, ty, yaw)
                if composed is None:
                    continue  # cadena no cerrable todavía (map→odom aún no llegó)
                x_m, y_m, yaw_m = composed
            if header_stamp:
                ts.append(tr.header.stamp.sec + tr.header.stamp.nanosec * 1e-9)
            else:
                ts.append(t_bag)
            xs.append(x_m)
            ys.append(y_m)
            yaws.append(yaw_m)

    return np.array(ts), np.array(xs), np.array(ys), np.array(yaws)


def extract_map_odom_offset(tf_entries, tf_static_entries):
    """Devuelve el offset (dx, dy, dyaw) de `map → odom` a partir del bag.

    Asume que `map → odom` es constante durante la corrida (lo típico en
    CoppeliaSim: la escena fija dónde arranca el robot y no toca esa TF
    después). Si no aparece, devuelve None y el caller asume identidad.
    """
    # Buscamos en /tf_static primero (caso ideal), después en /tf.
    for _, msg in tf_static_entries:
        for tr in msg.transforms:
            if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                return (tr.transform.translation.x,
                        tr.transform.translation.y,
                        _tf_yaw(tr))
    for _, msg in tf_entries:
        for tr in msg.transforms:
            if tr.header.frame_id == "map" and tr.child_frame_id == "odom":
                return (tr.transform.translation.x,
                        tr.transform.translation.y,
                        _tf_yaw(tr))
    return None


def _tf_yaw(tr):
    q = tr.transform.rotation
    return quat_to_yaw(q.x, q.y, q.z, q.w)


def apply_map_odom_offset(odom, offset):
    """Aplica el offset (dx, dy, dyaw) de map→odom a cada pose de la odometría.

    Devuelve una tupla igual a la que devuelve extract_odom pero con (x, y, yaw)
    expresados en frame `map`. Útil para el overlay en xy_path contra referencia
    y GT, que viven en `map`.
    """
    if offset is None:
        return odom
    dx, dy, dyaw = offset
    ts, x, y, yaw, vx, vy, wz = odom
    c, s = math.cos(dyaw), math.sin(dyaw)
    x_map = dx + c * x - s * y
    y_map = dy + s * x + c * y
    yaw_map = yaw + dyaw
    return (ts, x_map, y_map, yaw_map, vx, vy, wz)


def extract_wheel_cmd(entries):
    """Lista de (t, std_msgs/Float64) → (ts, values). Devuelve arrays vacíos si no hay entries."""
    if not entries:
        return np.array([]), np.array([])
    ts = np.array([t for t, _ in entries])
    values = np.array([m.data for _, m in entries])
    return ts, values


def plot_xy(odom, manifest, out_path: Path, ref_traj=None, gt=None, ekf=None, map_odom=None):
    ts, x, y, yaw, vx, vy, wz = odom
    fig, ax = plt.subplots(figsize=(6.5, 7))

    # Colores fijos en todas las corridas: referencia gris, odometría azul,
    # ground truth verde, EKF naranja (punteado, arriba de todo porque pisa al GT).
    C_REF, C_ODOM, C_GT, C_EKF = "0.45", "C0", "C2", "C1"

    xs_all = [np.asarray(x)]
    ys_all = [np.asarray(y)]

    # Referencia: trayectoria publicada (módulo 2/3) o círculo analítico (módulo 1).
    if ref_traj is not None:
        xr, yr, _ = ref_traj
        if len(xr):
            ax.plot(xr, yr, "--", color=C_REF, linewidth=1.2, label="referencia", zorder=1)
            xs_all.append(np.asarray(xr))
            ys_all.append(np.asarray(yr))
    else:
        # Círculo de radio R = vx/wz arrancando en el origen de `odom` con θ = 0
        # (centro en (0, R)); se lleva a `map` con el offset map→odom del sim.
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
                if map_odom is not None:
                    dx, dy, dyaw = map_odom
                    c, s = math.cos(dyaw), math.sin(dyaw)
                    x_ana, y_ana = dx + c * x_ana - s * y_ana, dy + s * x_ana + c * y_ana
                ax.plot(x_ana, y_ana, "--", color=C_REF, linewidth=1.2,
                        label=f"círculo teórico (R = {abs(R):.2f} m)", zorder=1)
                xs_all.append(x_ana)
                ys_all.append(y_ana)

    ax.plot(x, y, color=C_ODOM, linewidth=1.5, label="odometría", zorder=2)

    # Ground truth: pose real del simulador (base_link_gt).
    if gt is not None:
        ts_gt, xg, yg, _ = gt
        if len(xg):
            ax.plot(xg, yg, color=C_GT, linewidth=1.5, label="real (ground truth)", zorder=3)
            xs_all.append(np.asarray(xg))
            ys_all.append(np.asarray(yg))

    if ekf is not None and len(ekf["x"]):
        ax.plot(ekf["x"], ekf["y"], ":", color=C_EKF, linewidth=2.0, label="EKF", zorder=4)

    if len(x):
        ax.scatter([x[0]], [y[0]], marker="o", s=45, color="k", label="inicio", zorder=5)

    # Bbox cuadrado (evita que un recorrido angosto, ej. strafing, se estire),
    # con 10% de margen y un mínimo de 0.5 m.
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

    # Unas pocas flechas de orientación sobre la odometría (como el arrow de RViz).
    if len(x) > 1:
        n_arrows = min(8, len(x))
        idx = np.linspace(0, len(x) - 1, n_arrows + 1, dtype=int)[:-1]
        arrow_len = 0.07 * half
        ax.quiver(
            x[idx], y[idx], np.cos(yaw[idx]), np.sin(yaw[idx]),
            angles="xy", scale_units="xy", scale=1.0 / arrow_len,
            width=0.006, color=C_ODOM, alpha=0.8, zorder=2,
        )

    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.4)
    ax.set_title(f"Trayectoria (x, y) — {manifest.get('slug', '?')}")
    # Leyenda debajo del gráfico para no tapar el recorrido.
    ax.legend(loc="upper center", bbox_to_anchor=(0.5, -0.1), ncol=3, frameon=False)
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
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


# --- Módulo 3: EKF ---------------------------------------------------------------

def extract_ekf(entries):
    """/robot/odometry_ekf → dict con pose y covarianza, ordenado por stamp (sim-time).

    El nodo publica tras cada predicción (stamp de la odometría) y tras cada corrección
    (stamp del scan, que suele coincidir con uno ya publicado). Ordenamos por stamp y,
    ante stamps repetidos, nos quedamos con el último publicado: el corregido.
    """
    rows = {}
    for _, m in entries:
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        q = m.pose.pose.orientation
        c = m.pose.covariance  # 6x6 row-major sobre (x, y, z, roll, pitch, yaw)
        rows[t] = (m.pose.pose.position.x, m.pose.pose.position.y,
                   quat_to_yaw(q.x, q.y, q.z, q.w),
                   c[0], c[1], c[5], c[7], c[11], c[35])
    ts = np.array(sorted(rows))
    if not len(ts):
        return None
    a = np.array([rows[t] for t in ts])
    keys = ["x", "y", "yaw", "pxx", "pxy", "pxt", "pyy", "pyt", "ptt"]
    out = {k: a[:, i] for i, k in enumerate(keys)}
    out["t"] = ts
    return out


def odom_sim_time(entries, offset):
    """Pose de /robot/odometry en frame map, con el stamp del header (sim-time)."""
    ts = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for _, m in entries])
    o = apply_map_odom_offset(extract_odom(entries), offset)
    return ts, o[1], o[2], o[3]


def interp_pose(t_query, ts, xs, ys, yaws):
    """Interpola una trayectoria (x, y, yaw) en t_query; yaw se desenvuelve antes."""
    order = np.argsort(ts)
    ts, xs, ys, yaws = ts[order], xs[order], ys[order], yaws[order]
    return (np.interp(t_query, ts, xs), np.interp(t_query, ts, ys),
            np.interp(t_query, ts, np.unwrap(yaws)))


def wrap(a):
    return np.arctan2(np.sin(a), np.cos(a))


def ventana_movimiento(entries, umbral=0.01, margen=0.5):
    """(t_ini, t_fin) en sim-time en que el robot se mueve según /robot/odometry.

    Al terminar la trayectoria el robot queda quieto hasta que corta el watchdog: esas
    muestras no aportan al análisis del filtro y diluyen las métricas (p.ej. el NEES
    medio), así que se excluyen.
    """
    ts = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for _, m in entries])
    v = np.array([max(math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y),
                      abs(m.twist.twist.angular.z)) for _, m in entries])
    mov = np.where(v > umbral)[0]
    if not len(mov):
        return ts.min(), ts.max()
    return ts[mov[0]] - margen, ts[mov[-1]] + margen


def ekf_errors(ekf, gt_sim, odom_sim, ventana=None):
    """Errores EKF−GT y odom−GT en los instantes del EKF (dentro del rango del GT
    y, si se pasa, de la ventana (t_ini, t_fin))."""
    tg, xg, yg, thg = gt_sim
    t = ekf["t"]
    mask = (t >= tg.min()) & (t <= tg.max())
    if ventana is not None:
        mask &= (t >= ventana[0]) & (t <= ventana[1])
    t = t[mask]
    gx, gy, gth = interp_pose(t, tg, xg, yg, thg)
    e = {
        "t": t,
        "ex": ekf["x"][mask] - gx,
        "ey": ekf["y"][mask] - gy,
        "et": wrap(ekf["yaw"][mask] - gth),
    }
    for k in ["pxx", "pxy", "pxt", "pyy", "pyt", "ptt"]:
        e[k] = ekf[k][mask]
    to, xo, yo, tho = odom_sim
    ox, oy, oth = interp_pose(t, to, xo, yo, tho)
    e["odom_ex"], e["odom_ey"], e["odom_et"] = ox - gx, oy - gy, wrap(oth - gth)

    # NEES = eᵀ P⁻¹ e por muestra.
    nees = np.full(len(t), np.nan)
    for i in range(len(t)):
        P = np.array([[e["pxx"][i], e["pxy"][i], e["pxt"][i]],
                      [e["pxy"][i], e["pyy"][i], e["pyt"][i]],
                      [e["pxt"][i], e["pyt"][i], e["ptt"][i]]])
        v = np.array([e["ex"][i], e["ey"][i], e["et"][i]])
        try:
            nees[i] = float(v @ np.linalg.solve(P, v))
        except np.linalg.LinAlgError:
            pass
    e["nees"] = nees
    return e


def plot_ekf_vs_odom(e, out_path: Path, slug: str):
    t = e["t"] - e["t"][0]
    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)
    axes[0].plot(t, np.hypot(e["ex"], e["ey"]), color="C1", label="EKF")
    axes[0].plot(t, np.hypot(e["odom_ex"], e["odom_ey"]), color="C0", label="odometría")
    axes[0].set_ylabel("error de posición [m]")
    axes[1].plot(t, np.degrees(e["et"]), color="C1", label="EKF")
    axes[1].plot(t, np.degrees(e["odom_et"]), color="C0", label="odometría")
    axes[1].set_ylabel("error de yaw [°]")
    axes[1].set_xlabel("t sim [s]")
    for ax in axes:
        ax.grid(True)
        ax.legend()
    fig.suptitle(f"Error vs ground truth: EKF y odometría — {slug}")
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def plot_ekf_error_2sigma(e, out_path: Path, slug: str):
    t = e["t"] - e["t"][0]
    canales = [
        (e["ex"], np.sqrt(e["pxx"]), "error x [m]", 1.0),
        (e["ey"], np.sqrt(e["pyy"]), "error y [m]", 1.0),
        (e["et"], np.sqrt(e["ptt"]), "error θ [°]", 180.0 / math.pi),
    ]
    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for ax, (err, sig, name, k) in zip(axes, canales):
        ax.fill_between(t, -2 * sig * k, 2 * sig * k, color="C1", alpha=0.2, label="±2σ")
        ax.plot(t, err * k, color="C1", linewidth=1.0, label="error EKF")
        dentro = 100.0 * np.mean(np.abs(err) <= 2 * sig)
        # Escala: que se vea el error y la banda típica; los picos de σ (tramos sin
        # postes a la vista) quedan recortados en vez de aplastar todo el gráfico.
        lim = 1.5 * k * max(np.max(np.abs(err)), np.median(2 * sig))
        ax.set_ylim(-lim, lim)
        ax.set_ylabel(name)
        ax.set_title(f"{dentro:.0f}% de las muestras dentro de ±2σ", fontsize=9)
        ax.grid(True)
        ax.legend(loc="upper right")
    axes[-1].set_xlabel("t sim [s]")
    fig.suptitle(f"Error del EKF con bandas de ±2σ — {slug}")
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def plot_ekf_sigma(ekf_entries, landmarks, out_path: Path, slug: str, ventana=None,
                   zoom_s=3.0):
    """Desvío estándar estimado por el EKF en el tiempo.

    La incertidumbre oscila en serrucho: crece en cada predicción (odometría, 20 Hz) y
    cae en cada corrección con postes (~7 Hz). A la escala de toda la corrida eso es una
    mancha, así que se grafican las envolventes (máximo = antes de corregir, mínimo =
    después de corregir) en ventanas de 0.5 s, más un zoom de `zoom_s` segundos con los
    valores mensaje a mensaje. Se usan los mensajes en el orden en que se publicaron
    (sin deduplicar por stamp) para no perder las correcciones.
    """
    if not ekf_entries:
        return
    t = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for _, m in ekf_entries])
    c = [m.pose.covariance for _, m in ekf_entries]
    sxy = 100.0 * np.sqrt(0.5 * (np.array([x[0] for x in c]) + np.array([x[7] for x in c])))
    sth = np.degrees(np.sqrt(np.array([x[35] for x in c])))
    if ventana is not None:
        k = (t >= ventana[0]) & (t <= ventana[1])
        t, sxy, sth = t[k], sxy[k], sth[k]
    if not len(t):
        return
    t0 = t[0]
    tr = t - t0

    def envolventes(v, ancho=0.5):
        bordes = np.arange(0.0, tr[-1] + ancho, ancho)
        tc, vmin, vmax = [], [], []
        for b0 in bordes:
            k = (tr >= b0) & (tr < b0 + ancho)
            if k.any():
                tc.append(b0 + ancho / 2)
                vmin.append(v[k].min())
                vmax.append(v[k].max())
        return np.array(tc), np.array(vmin), np.array(vmax)

    n = 4 if landmarks else 3
    fig, axes = plt.subplots(n, 1, figsize=(9, 2.5 * n + 1))
    for ax, v, unidad, nombre in ((axes[0], sxy, "cm", "σ_xy"), (axes[1], sth, "°", "σ_θ")):
        tc, vmin, vmax = envolventes(v)
        ax.fill_between(tc, vmin, vmax, color="C1", alpha=0.15)
        ax.plot(tc, vmax, color="C3", label=f"{nombre} antes de corregir (máx.)")
        ax.plot(tc, vmin, color="C0", label=f"{nombre} después de corregir (mín.)")
        ax.set_ylabel(f"{nombre} [{unidad}]")
        ax.set_ylim(bottom=0)
        ax.grid(True)
        ax.legend(loc="upper right", fontsize=8)

    # Zoom en el medio de la corrida: valores mensaje a mensaje.
    zc = tr[-1] / 2
    k = (tr >= zc) & (tr <= zc + zoom_s)
    axes[2].plot(tr[k], sxy[k], "-o", color="C1", markersize=3, linewidth=1)
    axes[2].set_ylabel("σ_xy [cm]")
    axes[2].set_ylim(bottom=0)
    axes[2].set_title(f"Zoom de {zoom_s:.0f} s: crece en cada predicción, cae en cada corrección",
                      fontsize=9)
    axes[2].grid(True)
    axes[0].axvspan(zc, zc + zoom_s, color="0.5", alpha=0.25)

    if landmarks:
        tl = np.array([m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for _, m in landmarks])
        nl = np.array([len(m.landmarks) for _, m in landmarks])
        if ventana is not None:
            k = (tl >= ventana[0]) & (tl <= ventana[1])
            tl, nl = tl[k], nl[k]
        axes[3].step(tl - t0, nl, where="post", color="C7")
        axes[3].set_ylabel("postes por scan")
        axes[3].set_ylim(bottom=0)
        axes[3].grid(True)
        axes[3].set_xlim(axes[0].get_xlim())
    axes[1].set_xlim(axes[0].get_xlim())
    for ax in (axes[1], axes[-1]):
        ax.set_xlabel("t sim [s]")
    fig.suptitle(f"Incertidumbre estimada por el EKF en el tiempo — {slug}")
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def plot_ekf_elipses(ekf, gt, out_path: Path, slug: str, ventana=None, ref_traj=None,
                     n_elipses=16):
    """Trayectoria con elipses de covarianza (2σ, x-y) del EKF en puntos del recorrido.

    Equivalente al elipsoide de covarianza de RViz. Mientras el robot ve postes la
    incertidumbre es de pocos cm, invisible a la escala de la arena, así que las
    elipses se amplían por un factor que se indica en el título.
    """
    from matplotlib.patches import Ellipse

    t = ekf["t"]
    sel = np.ones(len(t), dtype=bool)
    if ventana is not None:
        sel = (t >= ventana[0]) & (t <= ventana[1])
    idx_all = np.where(sel)[0]
    if not len(idx_all):
        return
    idx = idx_all[np.linspace(0, len(idx_all) - 1, n_elipses, dtype=int)]

    # Semiejes 2σ de cada elipse (autovalores de la covarianza x-y).
    elipses = []
    for i in idx:
        P = np.array([[ekf["pxx"][i], ekf["pxy"][i]], [ekf["pxy"][i], ekf["pyy"][i]]])
        val, vec = np.linalg.eigh(P)
        val = np.clip(val, 0.0, None)
        ang = math.degrees(math.atan2(vec[1, 1], vec[0, 1]))
        elipses.append((ekf["x"][i], ekf["y"][i], 2 * np.sqrt(val[1]), 2 * np.sqrt(val[0]), ang))

    # Factor de ampliación "redondo" para que la elipse típica mida ~10 cm de semieje.
    semieje = np.median([e[2] for e in elipses])
    escala = 1
    for f in (1, 2, 5, 10, 20, 50, 100, 200, 500):
        escala = f
        if semieje * f >= 0.10:
            break

    fig, ax = plt.subplots(figsize=(7, 7))
    if ref_traj is not None:
        ax.plot(ref_traj[0], ref_traj[1], "k--", linewidth=1, label="referencia")
    if gt is not None:
        ax.plot(gt[1], gt[2], color="C2", linewidth=1.2, label="ground truth")
    ax.plot(ekf["x"][sel], ekf["y"][sel], color="C1", linewidth=1.0, label="EKF")
    for k, (x, y, a, b, ang) in enumerate(elipses):
        ax.add_patch(Ellipse((x, y), 2 * a * escala, 2 * b * escala, angle=ang,
                             fill=True, alpha=0.25, color="C1",
                             label=f"elipse 2σ (×{escala})" if k == 0 else None))
        ax.plot(x, y, ".", color="C1")
    ax.scatter(POSTES_XY[:, 0], POSTES_XY[:, 1], marker="s", s=30, color="0.3", label="postes")
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True, alpha=0.4)
    ax.legend(loc="lower left", fontsize=8, framealpha=0.9)
    ax.set_title(f"Covarianza del EKF a lo largo del recorrido — elipses 2σ ampliadas ×{escala}\n{slug}",
                 fontsize=10)
    fig.tight_layout()
    fig.savefig(out_path, dpi=130)
    plt.close(fig)


def plot_ekf_nees(e, out_path: Path, slug: str):
    t = e["t"] - e["t"][0]
    nees = e["nees"]
    ok = np.isfinite(nees) & (nees > 0)
    fig, ax = plt.subplots(figsize=(9, 4))
    ax.semilogy(t[ok], nees[ok], color="C1", linewidth=0.8, label="NEES")
    ax.axhline(3.0, color="k", linestyle="--", linewidth=1, label="valor esperado (3)")
    ax.axhspan(NEES_CHI2_3_LO, NEES_CHI2_3_HI, color="C2", alpha=0.15,
               label="intervalo 95% χ²₃")
    ax.set_xlabel("t sim [s]")
    ax.set_ylabel("eᵀ P⁻¹ e")
    ax.set_title(f"NEES — media {np.nanmean(nees):.2f} — {slug}")
    ax.grid(True, which="both", alpha=0.4)
    ax.legend(loc="upper right")
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def ekf_metrics(e):
    pe = np.hypot(e["ex"], e["ey"])
    po = np.hypot(e["odom_ex"], e["odom_ey"])
    nees = e["nees"][np.isfinite(e["nees"])]
    f = lambda v: round(float(v), 4)  # noqa: E731
    return {
        "ekf": {
            "error_pos_medio_m": f(pe.mean()), "error_pos_max_m": f(pe.max()),
            "error_pos_final_m": f(pe[-1]),
            "error_yaw_medio_deg": f(np.degrees(np.abs(e["et"])).mean()),
            "error_yaw_max_deg": f(np.degrees(np.abs(e["et"])).max()),
            "error_yaw_final_deg": f(np.degrees(e["et"][-1])),
        },
        "odometria": {
            "error_pos_medio_m": f(po.mean()), "error_pos_max_m": f(po.max()),
            "error_pos_final_m": f(po[-1]),
            "error_yaw_final_deg": f(np.degrees(e["odom_et"][-1])),
        },
        "consistencia": {
            "dentro_2sigma_x": f(np.mean(np.abs(e["ex"]) <= 2 * np.sqrt(e["pxx"]))),
            "dentro_2sigma_y": f(np.mean(np.abs(e["ey"]) <= 2 * np.sqrt(e["pyy"]))),
            "dentro_2sigma_theta": f(np.mean(np.abs(e["et"]) <= 2 * np.sqrt(e["ptt"]))),
            "nees_medio": f(nees.mean()),
            "nees_dentro_intervalo_95": f(np.mean((nees >= NEES_CHI2_3_LO) & (nees <= NEES_CHI2_3_HI))),
            "sigma_xy_medio_m": f(np.sqrt(0.5 * (e["pxx"] + e["pyy"])).mean()),
            "sigma_theta_medio_deg": f(np.degrees(np.sqrt(e["ptt"])).mean()),
        },
    }


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
    topics += list(TF_TOPICS)  # para reconstruir la pose GT desde /tf.
    topics += [EKF_TOPIC, LANDMARKS_TOPIC]  # módulo 3 (si no están en el bag, quedan vacíos)
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

    tf_dyn = msgs.get("/tf", [])
    tf_sta = msgs.get("/tf_static", [])
    gt = extract_gt_pose(tf_dyn, tf_sta)
    if not len(gt[0]):
        print(
            "Aviso: no se encontró ninguna TF con child='base_link_gt' en el bag — el "
            "overlay de ground truth no se dibuja. Agregá /tf y /tf_static a "
            "topics_grabados del manifest si lo querés.",
            file=sys.stderr,
        )
        gt = None

    # Trasladamos la odometría al frame `map` para que el plot sea comparable con
    # la referencia (viene en map) y el GT (idem). Si map→odom no se encuentra,
    # se asume identidad.
    map_odom = extract_map_odom_offset(tf_dyn, tf_sta)
    if map_odom is None:
        print(
            "Aviso: no se encontró TF `map → odom` en el bag — se grafica la "
            "odometría en frame `odom` (origen en el inicio de la corrida). Si la "
            "referencia es en `map`, va a aparecer con un offset constante.",
            file=sys.stderr,
        )
    odom_in_map = apply_map_odom_offset(odom, map_odom)

    enc_wheels = extract_wheel_encoder_velocities(msgs.get(ENCODER_TOPIC, []))
    cmd_wheels = {i: extract_wheel_cmd(msgs.get(topic, []))
                  for i, topic in enumerate(WHEEL_CMD_TOPICS)}
    if not any(len(v[0]) for v in cmd_wheels.values()):
        print(
            "Aviso: el bag no contiene /robot/*_wheel/cmd_vel — sólo se plotea ω medido. "
            "Agregá esos topics a topics_grabados para ver la comparación medido vs comandado.",
            file=sys.stderr,
        )

    ekf = extract_ekf(msgs.get(EKF_TOPIC, []))

    print("Generando plots...")
    # xy_path y los plots de módulo 2 (tracking_error, yaw_error) usan pose en
    # map: así la odometría, la referencia y el GT caen en el mismo frame y las
    # distancias tienen sentido físico. El resto (body_velocity, odom_vs_cmd,
    # wheel_velocities) vive en el marco del robot/odom y no cambia.
    plot_xy(odom_in_map, manifest, plots_dir / "xy_path.png", ref_traj=ref_traj, gt=gt, ekf=ekf,
            map_odom=map_odom)

    if ekf is not None:
        if gt is None:
            print("Aviso: hay EKF pero no GT en el bag — sólo se grafica σ.", file=sys.stderr)
            plot_ekf_sigma(msgs[EKF_TOPIC], msgs.get(LANDMARKS_TOPIC, []), plots_dir / "ekf_sigma.png", slug)
        else:
            # Todo en sim-time (stamps de header) para comparar muestra a muestra.
            gt_sim = extract_gt_pose(tf_dyn, tf_sta, header_stamp=True)
            odom_sim = odom_sim_time(msgs["/robot/odometry"], map_odom)
            ventana = ventana_movimiento(msgs["/robot/odometry"])
            e = ekf_errors(ekf, gt_sim, odom_sim, ventana)
            plot_ekf_vs_odom(e, plots_dir / "ekf_vs_odom_error.png", slug)
            plot_ekf_error_2sigma(e, plots_dir / "ekf_error_2sigma.png", slug)
            plot_ekf_sigma(msgs[EKF_TOPIC], msgs.get(LANDMARKS_TOPIC, []), plots_dir / "ekf_sigma.png",
                           slug, ventana)
            plot_ekf_nees(e, plots_dir / "ekf_nees.png", slug)
            gt_xy = extract_gt_pose(tf_dyn, tf_sta)
            plot_ekf_elipses(ekf, gt_xy, plots_dir / "ekf_elipses.png", slug, ventana,
                             ref_traj=ref_traj)
            metricas = ekf_metrics(e)
            metricas["ventana_analizada_s"] = round(float(e["t"][-1] - e["t"][0]), 1)
            with (plots_dir / "metricas_ekf.yaml").open("w") as f:
                yaml.safe_dump(metricas, f, sort_keys=False, allow_unicode=True)
            print(yaml.safe_dump(metricas, sort_keys=False, allow_unicode=True))
    plot_body_velocity(odom, cmd, plots_dir / "body_velocity.png", slug)
    plot_odom_vs_cmd(odom, cmd, plots_dir / "odom_vs_cmd.png", slug)
    plot_wheel_velocities(enc_wheels, cmd_wheels, plots_dir / "wheel_velocities.png", slug)

    if closed_loop and ref_traj is not None:
        plot_tracking_error(odom_in_map, ref_traj, plots_dir / "tracking_error.png", slug)
        plot_yaw_error(odom_in_map, ref_traj, plots_dir / "yaw_error.png", slug)
    elif closed_loop and ref_traj is None:
        print(
            f"Aviso: cmd_source es closed_loop_launch pero el bag no tiene {REF_TRAJ_TOPIC} "
            f"— agregalo a topics_grabados para habilitar tracking_error / yaw_error.",
            file=sys.stderr,
        )

    if closed_loop and goals is not None and len(goals[0]):
        # goal_pose viene en map; comparamos contra la odometría también en map.
        plot_goal_pose_vs_pose(odom_in_map, goals, plots_dir / "goal_pose_vs_pose.png", slug)

    print(f"Listo: {plots_dir}")


if __name__ == "__main__":
    main()
