#!/usr/bin/env bash
# Unpack a project's images and process them with OpenDroneMap on macOS.
set -euo pipefail

if [[ ${1:-} == --help || $# -gt 1 ]]; then
  echo "Usage: $0 [project-directory]"
  echo "Defaults to the current directory. The directory must contain images.zip."
  echo "ODM output is always saved in the project's images/ directory."
  echo "Layering is attempted when Python, Pillow, and a JPG or PNG GPS mosaic are available."
  exit 0
fi

project_dir=${1:-.}

if [[ ! -d $project_dir ]]; then
  echo "Project directory does not exist: $project_dir" >&2
  exit 1
fi

project_dir=$(cd -- "$project_dir" && pwd -P)
archive="$project_dir/images.zip"
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)
layer_script="$script_dir/layer_images.py"

if [[ ! -f $archive ]]; then
  echo "Missing image archive: $archive" >&2
  exit 1
fi

command -v unzip >/dev/null 2>&1 || {
  echo "unzip is required but was not found." >&2
  exit 1
}

command -v docker >/dev/null 2>&1 || {
  echo "Docker is required but was not found. Install and start Docker Desktop." >&2
  exit 1
}

odm_dir="$project_dir/images"
input_dir="$odm_dir/images"
mkdir -p "$input_dir"
echo "Extracting $archive into $input_dir"
unzip -o "$archive" -d "$input_dir"

echo "Running OpenDroneMap; results will be written to $odm_dir"
docker run -ti --rm \
  -v "$odm_dir:/datasets/code" \
  opendronemap/odm \
  --project-path /datasets \
  --orthophoto-resolution 2.2 \
  --orthophoto-png \
  --rerun-all

ortho_png="$odm_dir/odm_orthophoto/odm_orthophoto.png"
if [[ ! -f $ortho_png ]]; then
  fallback_ortho_png="$odm_dir/opensfm/stats/ortho.png"
  if [[ -f $fallback_ortho_png ]]; then
    ortho_png=$fallback_ortho_png
    echo "Using fallback orthophoto: $ortho_png"
  else
    echo "ODM completed, but no orthophoto PNG was found." >&2
    echo "Checked: $ortho_png" >&2
    echo "Checked: $fallback_ortho_png" >&2
    exit 1
  fi
fi

tosu_png="$odm_dir/TOSU.png"
cp "$ortho_png" "$tosu_png"
echo "Copied orthophoto to $tosu_png"

if ! command -v python3 >/dev/null 2>&1; then
  echo "Warning: python3 was not found; skipping layered image." >&2
elif [[ ! -f $layer_script ]]; then
  echo "Warning: image layering script was not found; skipping layered image." >&2
elif ! python3 -c 'import PIL' >/dev/null 2>&1; then
  echo "Warning: Python package Pillow was not found; skipping layered image." >&2
else
  gps_mosaic=""
  for candidate in \
    "$project_dir"/gps_mosaic_*.jpg \
    "$project_dir"/gps_mosaic_*.png; do
    [[ -f $candidate ]] || continue
    if [[ -z $gps_mosaic || $candidate -nt $gps_mosaic ]]; then
      gps_mosaic=$candidate
    fi
  done

  if [[ -z $gps_mosaic ]]; then
    echo "Warning: no gps_mosaic_*.jpg or gps_mosaic_*.png was found; skipping layered image." >&2
  else
    layered_png="$odm_dir/TOSU_layered.png"
    echo "Layering $tosu_png over newest GPS mosaic: $gps_mosaic"
    if python3 "$layer_script" "$gps_mosaic" "$tosu_png" "$layered_png"; then
      echo "Saved layered orthophoto to $layered_png"
    else
      echo "Warning: layering failed, but the ODM output remains saved at $tosu_png" >&2
    fi
  fi
fi
