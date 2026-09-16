#!/usr/bin/env bash
# Unpack a project's images and process them with OpenDroneMap on macOS.
set -euo pipefail

if [[ ${1:-} == --help || $# -gt 1 ]]; then
  echo "Usage: $0 [project-directory]"
  echo "Defaults to the current directory. The directory must contain images.zip."
  echo "ODM output is written to the project's images/ directory."
  exit 0
fi

project_dir=${1:-.}

if [[ ! -d $project_dir ]]; then
  echo "Project directory does not exist: $project_dir" >&2
  exit 1
fi

project_dir=$(cd -- "$project_dir" && pwd -P)
archive="$project_dir/images.zip"

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
  --orthophoto-png

ortho_png="$odm_dir/odm_orthophoto/odm_orthophoto.png"
if [[ ! -f $ortho_png ]]; then
  echo "ODM completed, but expected PNG was not found: $ortho_png" >&2
  exit 1
fi

cp "$ortho_png" "$odm_dir/TOSU.png"
echo "Copied orthophoto to $odm_dir/TOSU.png"
