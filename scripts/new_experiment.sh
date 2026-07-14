#!/usr/bin/env bash
# Bootstrap de una corrida experimental nueva bajo experiments/YYYY-MM-DD_<slug>/.
#
# Uso: bash scripts/new_experiment.sh <slug> "<descripción breve>"
#
# Crea el directorio, copia el template y rellena los campos autocompletables
# del manifest (slug, fecha, git_sha, git_dirty, descripcion).

set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "Uso: $0 <slug> \"<descripción>\"" >&2
  echo "Ejemplo: $0 strafing_canary \"canary de strafing puro post-fix\"" >&2
  exit 1
fi

slug="$1"
desc="$2"

repo_root="$(git rev-parse --show-toplevel)"
cd "$repo_root"

fecha="$(date -I)"
sha="$(git rev-parse --short HEAD)"
if [[ -n "$(git status --porcelain)" ]]; then
  dirty="true"
else
  dirty="false"
fi

dir="experiments/${fecha}-${slug}"
if [[ -e "$dir" ]]; then
  echo "Ya existe $dir — abortando para no pisar una corrida." >&2
  exit 1
fi

mkdir -p "$dir/plots"
cp experiments/template/manifest.yaml "$dir/manifest.yaml"
cp experiments/template/notes.md "$dir/notes.md"

# Escapar caracteres problemáticos para sed en la descripción.
desc_escaped="${desc//\\/\\\\}"
desc_escaped="${desc_escaped//&/\\&}"
desc_escaped="${desc_escaped//|/\\|}"

sed -i \
  -e "s|^slug:.*|slug: ${slug}|" \
  -e "s|^fecha:.*|fecha: ${fecha}|" \
  -e "s|^git_sha:.*|git_sha: ${sha}|" \
  -e "s|^git_dirty:.*|git_dirty: ${dirty}|" \
  -e "s|^descripcion:.*|descripcion: ${desc_escaped}|" \
  "$dir/manifest.yaml"

echo "Creado: $dir"
echo ""
echo "Próximos pasos:"
echo "  1. Editá $dir/manifest.yaml — completá launch, trayectoria, cmd_source, duracion_maxima_s."
echo "  2. Con el simulador levantado, dentro del container (source install/setup.bash):"
echo "       bash scripts/run_experiment.sh $dir"
echo "  3. Post-corrida: python3 scripts/plot_run.py $dir"
