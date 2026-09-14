#!/usr/bin/env bash
# Fast-forward all three Jetson repos to their local committed HEADs, without GitHub.
set -euo pipefail

if [[ ${1:-} == --help || $# -gt 2 ]]; then
  echo "Usage: $0 [ssh-host] [remote-repo]"
  echo "Defaults: whichever of bvorinnano@192.168.55.1 (USB-C) or"
  echo "bvorinnano@192.168.144.2 (Herelink) answers; ~/bv_ws/src/bv_core on the Jetson"
  echo "Syncs bv_core, bv_gcs, and bv_msgs; the other repos are siblings of bv_core."
  echo "Transfers committed HEAD only; leaves local uncommitted edits alone."
  exit 0
fi

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
source "$script_dir/jetson_host.sh"
jetson_host=$(find_jetson "${1:-}")
remote_repo=${2:-bv_ws/src/bv_core}
core_repo=$(git -C "$script_dir" rev-parse --show-toplevel)
workspace_dir=$(dirname -- "$core_repo")
remote_workspace=$(dirname -- "${remote_repo%/}")
sync_dir=$(mktemp -d)
trap 'rm -rf -- "$sync_dir"' EXIT

# Prepare every bundle before updating any remote repository.
repos=(bv_core bv_gcs bv_msgs)
branches=()
commits=()
for repo in "${repos[@]}"; do
  repo_dir="$workspace_dir/$repo"
  if [[ $repo == bv_core ]]; then
    repo_dir=$core_repo
  fi
  branch=$(git -C "$repo_dir" symbolic-ref --quiet --short HEAD) || {
    echo "Cannot sync $repo; ensure it exists and has a branch checked out." >&2
    exit 1
  }
  branches+=("$branch")
  commits+=("$(git -C "$repo_dir" rev-parse HEAD)")
  git -C "$repo_dir" bundle create "$sync_dir/$repo.bundle" "refs/heads/$branch"
done

remote_script=$(cat <<'REMOTE'
set -euo pipefail
repo=$1
branch=$2
expected=$3
# Relative paths are relative to the remote login home.
cd -- "$repo"
if [[ -n $(git status --porcelain --untracked-files=all) ]]; then
  echo "Jetson has uncommitted or untracked files; commit or stash them first." >&2
  exit 1
fi
actual_branch=$(git symbolic-ref --quiet --short HEAD) || {
  echo "Jetson has a detached HEAD; check out the matching branch first." >&2
  exit 1
}
if [[ $actual_branch != "$branch" ]]; then
  echo "Branch mismatch: Mac=$branch, Jetson=$actual_branch. No update made." >&2
  exit 1
fi
sync_dir=$(mktemp -d)
trap 'rm -rf -- "$sync_dir"' EXIT
cat > "$sync_dir/repo.bundle"
git fetch --no-tags "$sync_dir/repo.bundle" "refs/heads/$branch"
received=$(git rev-parse FETCH_HEAD)
if [[ $received != "$expected" ]]; then
  echo "Mac branch changed during transfer; rerun the sync." >&2
  exit 1
fi
if ! git merge-base --is-ancestor HEAD "$received"; then
  echo "Jetson has commits absent from the Mac; refusing to overwrite them." >&2
  exit 1
fi
git merge --ff-only "$received"
echo "Jetson synced: $branch at $(git rev-parse --short HEAD)"
REMOTE
)

for i in "${!repos[@]}"; do
  repo=${repos[$i]}
  branch=${branches[$i]}
  commit=${commits[$i]}
  target_repo="$remote_workspace/$repo"
  if [[ $repo == bv_core ]]; then
    target_repo=$remote_repo
  fi
  # SSH passes its command through the remote shell; quote every argument.
  printf -v remote_command 'bash -c %q -- %q %q %q' \
    "$remote_script" "$target_repo" "$branch" "$commit"
  echo "Syncing $jetson_host:$target_repo to $branch at ${commit:0:7}"
  ssh "$jetson_host" "$remote_command" < "$sync_dir/$repo.bundle"
done
