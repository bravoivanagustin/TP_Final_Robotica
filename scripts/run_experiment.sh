#!/usr/bin/env bash
# Orquestador de una corrida experimental.
#
# Lanza (1) el backend declarado en manifest.launch, (2) la grabación con record.sh,
# (3) el driver dispatched por manifest.cmd_source.tipo. Espera a que el driver termine
# (o al watchdog `duracion_maxima_s`), después flushea el bag y mata todo.
#
# El simulador NO se lanza desde acá: es pesado, visual, requiere X11 y su vida útil
# suele exceder una corrida. Asumimos que ya está corriendo en otra terminal.
#
# Uso: bash scripts/run_experiment.sh <experiments/YYYY-MM-DD_slug>
# Correr DENTRO del contenedor Docker, con `source install/setup.bash` ya hecho.

set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Uso: $0 <experiments/YYYY-MM-DD_slug>" >&2
  exit 1
fi

dir="$1"
manifest="$dir/manifest.yaml"

if [[ ! -f "$manifest" ]]; then
  echo "No encontré $manifest — ¿corriste new_experiment.sh?" >&2
  exit 1
fi

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# --- Helpers de parseo del manifest ---
# Delegamos a python3 + pyyaml para no reinventar YAML en bash/awk.

get_field() {
  # Uso: get_field <dotted.key>  → imprime el valor (o vacío si falta)
  MANIFEST="$manifest" KEY="$1" python3 - <<'PY'
import os, yaml
with open(os.environ["MANIFEST"]) as f:
    m = yaml.safe_load(f) or {}
node = m
for k in os.environ["KEY"].split("."):
    if isinstance(node, dict):
        node = node.get(k)
    else:
        node = None
        break
print("" if node is None else node)
PY
}

get_launch_args() {
  # Uso: get_launch_args <dotted.key>  → imprime "k1:=v1 k2:=v2" para el dict debajo de esa key.
  # Booleans YAML se serializan como "true"/"false" en minúscula (lo que espera ros2 launch).
  MANIFEST="$manifest" KEY="$1" python3 - <<'PY'
import os, shlex, yaml
with open(os.environ["MANIFEST"]) as f:
    m = yaml.safe_load(f) or {}
node = m
for k in os.environ["KEY"].split("."):
    node = node.get(k) if isinstance(node, dict) else None
if not isinstance(node, dict):
    node = {}
parts = []
for k, v in node.items():
    if isinstance(v, bool):
        s = "true" if v else "false"
    else:
        s = str(v)
    parts.append(f"{k}:={shlex.quote(s)}")
print(" ".join(parts))
PY
}

backend_paquete="$(get_field launch.paquete)"
backend_archivo="$(get_field launch.archivo)"
backend_args="$(get_launch_args launch.args)"

cmd_tipo="$(get_field cmd_source.tipo)"
cmd_paquete="$(get_field cmd_source.paquete)"
cmd_archivo="$(get_field cmd_source.archivo)"
cmd_args="$(get_launch_args cmd_source.args)"

duracion_max="$(get_field duracion_maxima_s)"
: "${duracion_max:=120}"

if [[ -z "$cmd_tipo" ]]; then
  echo "El manifest no declara cmd_source.tipo — abortando." >&2
  exit 1
fi

# --- Cleanup de hijos ---
# `ros2 launch` no siempre propaga SIGINT a sus nodos nietos → zombies. Lo lanzamos con
# `setsid` para que cada launch ocupe su propio grupo de procesos, y matamos con
# `kill -SIGNAL -PID` (PID negativo → todo el grupo).
PIDS=()
cleanup_ran=false
cleanup() {
  if $cleanup_ran; then return; fi
  cleanup_ran=true
  echo ""
  echo "[cleanup] Deteniendo procesos hijos (grupo)..."
  for pid in "${PIDS[@]:-}"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      # Grupo primero; si el proceso no es líder de grupo (ej. record.sh sin setsid), PID directo.
      kill -INT "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
    fi
  done
  # Damos 3 s para que rosbag2 flushee y ros2 launch cierre prolijo.
  for _ in 1 2 3; do
    all_done=true
    for pid in "${PIDS[@]:-}"; do
      if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
        all_done=false
      fi
    done
    $all_done && break
    sleep 1
  done
  # Segunda pasada: SIGTERM al grupo para lo que sobreviva.
  for pid in "${PIDS[@]:-}"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -TERM "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    fi
  done
}
trap cleanup INT TERM EXIT

t_start=$(date +%s)

# --- 1. Backend ---
if [[ -n "$backend_paquete" && -n "$backend_archivo" ]]; then
  echo "[1/3] Backend: setsid ros2 launch $backend_paquete $backend_archivo $backend_args"
  # shellcheck disable=SC2086
  setsid ros2 launch "$backend_paquete" "$backend_archivo" $backend_args &
  PID_BACK=$!
  PIDS+=("$PID_BACK")
  sleep 2
else
  echo "[1/3] Backend: (ninguno declarado)"
fi

# --- 2. Grabación ---
echo "[2/3] Grabación: bash scripts/record.sh $dir"
bash "$script_dir/record.sh" "$dir" &
PID_REC=$!
PIDS+=("$PID_REC")
sleep 2

# --- 3. Driver ---
echo "[3/3] Driver: $cmd_tipo (watchdog=${duracion_max}s)"
case "$cmd_tipo" in
  open_loop_twist)
    # publish_cmd_vel.py termina solo cuando llega a duracion_s.
    python3 "$script_dir/publish_cmd_vel.py" "$dir" || true
    ;;

  closed_loop_launch)
    if [[ -z "$cmd_paquete" || -z "$cmd_archivo" ]]; then
      echo "cmd_source.tipo == closed_loop_launch requiere cmd_source.paquete y cmd_source.archivo." >&2
      exit 1
    fi
    # shellcheck disable=SC2086
    setsid ros2 launch "$cmd_paquete" "$cmd_archivo" $cmd_args &
    PID_DRV=$!
    PIDS+=("$PID_DRV")
    # Watchdog en background — mata el grupo del driver si excede duracion_max.
    ( sleep "$duracion_max" && kill -INT "-$PID_DRV" 2>/dev/null && \
        echo "[watchdog] duracion_maxima_s=${duracion_max} alcanzado — matando driver." >&2 ) &
    PID_WATCH=$!
    wait "$PID_DRV" 2>/dev/null || true
    kill "$PID_WATCH" 2>/dev/null || true
    ;;

  manual)
    echo "[driver] Modo manual. Publicá /robot/cmd_vel desde otra terminal (teleop, ros2 topic pub, ...)"
    echo "[driver] Cuando termines, presioná Enter acá para cerrar la grabación."
    read -r || true
    ;;

  *)
    echo "cmd_source.tipo desconocido: '$cmd_tipo' (esperaba open_loop_twist|closed_loop_launch|manual)" >&2
    exit 1
    ;;
esac

# --- Cierre ordenado ---
# El trap va a mandar SIGINT a record.sh y backend; rosbag2 flushea en el INT handler.
echo ""
echo "[driver] Terminado. Cerrando grabación y backend..."
cleanup

t_end=$(date +%s)
dur=$((t_end - t_start))
bag_size="?"
if [[ -d "$dir/bag" ]]; then
  bag_size=$(du -sh "$dir/bag" 2>/dev/null | cut -f1)
fi

echo ""
echo "═══════════════════════════════════════════════════════════════"
echo " Corrida terminada: $dir"
echo " Duración total:    ${dur}s"
echo " Tamaño del bag:    $bag_size"
echo " Siguiente paso:    python3 scripts/plot_run.py $dir"
echo "═══════════════════════════════════════════════════════════════"
