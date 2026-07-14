#!/usr/bin/env bash
# Wrapper de `ros2 bag record` que lee la lista de topics del manifest.yaml.
#
# Uso: bash scripts/record.sh <dir-de-corrida> [args-extra-para-rosbag2...]
#
# Nota: correr DENTRO del contenedor Docker con `source install/setup.bash` ya hecho.

set -euo pipefail

if [[ $# -lt 1 ]]; then
  echo "Uso: $0 <experiments/YYYY-MM-DD_slug> [args extra para ros2 bag record]" >&2
  exit 1
fi

dir="$1"; shift || true
manifest="$dir/manifest.yaml"

if [[ ! -f "$manifest" ]]; then
  echo "No encontré $manifest — ¿corriste new_experiment.sh?" >&2
  exit 1
fi

# Parseo simple del block-list "topics_grabados: \n  - /topic1 \n  - /topic2".
mapfile -t topics < <(awk '
  /^topics_grabados:/ { grab=1; next }
  grab && /^[^[:space:]]/ { grab=0 }
  grab && /^[[:space:]]*-[[:space:]]*/ {
    sub(/^[[:space:]]*-[[:space:]]*/, "");
    sub(/[[:space:]]+#.*$/, "");
    print
  }
' "$manifest")

if [[ ${#topics[@]} -eq 0 ]]; then
  echo "manifest.yaml no lista topics en topics_grabados — abortando." >&2
  exit 1
fi

if [[ -f "$dir/bag/metadata.yaml" ]]; then
  echo "Ya hay un bag grabado en $dir/bag — moverlo/borrarlo antes de re-grabar." >&2
  exit 1
fi
# rosbag2 quiere crear el directorio de salida él mismo.
rm -rf "$dir/bag"

echo "Grabando topics: ${topics[*]}"
echo "Salida: $dir/bag/"
echo "Ctrl-C para detener."

exec ros2 bag record -o "$dir/bag" "$@" "${topics[@]}"
