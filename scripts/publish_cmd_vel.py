#!/usr/bin/env python3
"""Publica /robot/cmd_vel constante leyendo manifest.yaml.

Se usa sólo en módulo 1 (open-loop). Lo invoca scripts/run_experiment.sh cuando
`cmd_source.tipo == open_loop_twist`. Publica el mismo Twist a 20 Hz durante
`trayectoria.parametros.duracion_s` segundos y termina publicando ceros.

Uso: python3 scripts/publish_cmd_vel.py <run_dir>
"""

import argparse
import sys
import time
from pathlib import Path

import yaml

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from geometry_msgs.msg import Twist
except ImportError as e:
    print(f"Faltan libs de ROS 2 ({e}). Corré `source install/setup.bash` primero.", file=sys.stderr)
    sys.exit(1)


PUBLISH_RATE_HZ = 20.0


def build_twist(tipo: str, parametros: dict) -> Twist:
    """Arma el Twist constante según el tipo de trayectoria."""
    p = parametros or {}
    tw = Twist()
    if tipo == "strafing":
        tw.linear.y = float(p.get("vy", 0.0))
    elif tipo == "circular":
        tw.linear.x = float(p.get("vx", 0.0))
        tw.angular.z = float(p.get("wz", 0.0))
    elif tipo == "constante":
        tw.linear.x = float(p.get("vx", 0.0))
        tw.linear.y = float(p.get("vy", 0.0))
        tw.angular.z = float(p.get("wz", 0.0))
    else:
        print(
            f"trayectoria.tipo no soportado en open-loop: {tipo!r}. "
            "Tipos válidos: strafing, circular, constante.",
            file=sys.stderr,
        )
        sys.exit(2)
    return tw


class ExperimentCmdPublisher(Node):
    def __init__(self, twist: Twist, duracion_s: float):
        # Forzamos use_sim_time=True para que get_clock() devuelva sim-time. Sin esto,
        # el timing sería wall clock y si el sim corre a factor != 1× perderíamos parte
        # de la trayectoria (o la excederíamos). "duracion_s" del manifest se interpreta
        # en sim-time (que es lo que ve el robot).
        super().__init__(
            "experiment_cmd_publisher",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self.pub = self.create_publisher(Twist, "/robot/cmd_vel", 10)
        self.twist = twist
        self.duracion_s = duracion_s
        # t0 se setea en el primer callback — cuando /clock ya haya llegado y sim-time
        # tenga un valor válido.
        self.t0 = None
        self._done = False
        self.timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self._on_timer)

    def _on_timer(self):
        if self._done:
            return
        now = self.get_clock().now()
        if self.t0 is None:
            self.t0 = now
            self.pub.publish(self.twist)
            return
        elapsed = (now - self.t0).nanoseconds * 1e-9
        if elapsed >= self.duracion_s:
            self.get_logger().info(
                f"Duración {self.duracion_s:.2f}s (sim) alcanzada — parando."
            )
            self._done = True
            self.timer.cancel()
            return
        self.pub.publish(self.twist)

    def is_done(self) -> bool:
        return self._done


def main():
    parser = argparse.ArgumentParser(description="Publica cmd_vel constante para open-loop.")
    parser.add_argument("run_dir", type=Path, help="Directorio de la corrida (experiments/<fecha>_<slug>)")
    args = parser.parse_args()

    manifest_path = args.run_dir / "manifest.yaml"
    if not manifest_path.exists():
        print(f"No existe {manifest_path}", file=sys.stderr)
        sys.exit(1)

    with manifest_path.open() as f:
        manifest = yaml.safe_load(f) or {}

    tray = manifest.get("trayectoria") or {}
    tipo = tray.get("tipo")
    if not tipo:
        print("El manifest no declara trayectoria.tipo — no sé qué publicar.", file=sys.stderr)
        sys.exit(2)
    parametros = tray.get("parametros") or {}
    duracion_s = float(parametros.get("duracion_s", 5.0))

    twist = build_twist(tipo, parametros)

    rclpy.init()
    node = ExperimentCmdPublisher(twist, duracion_s)

    node.get_logger().info(
        f"Publicando tipo={tipo!r} durante {duracion_s:.2f}s a {PUBLISH_RATE_HZ} Hz."
    )
    try:
        # spin_once con timeout corto: chequeamos la condición de salida desde el main
        # thread en vez de llamar shutdown desde el timer callback (que puede colgar la
        # spin loop). rclpy maneja SIGINT nativamente y lo convierte en KeyboardInterrupt.
        while rclpy.ok() and not node.is_done():
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass

    # Zero-twist explícito desde el main thread — el executor ya no dispara el timer.
    if rclpy.ok():
        node.get_logger().info("Publicando zero-twist final y saliendo.")
        zero = Twist()
        for _ in range(3):
            node.pub.publish(zero)
            time.sleep(0.05)

    try:
        node.destroy_node()
    except Exception:
        pass
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
