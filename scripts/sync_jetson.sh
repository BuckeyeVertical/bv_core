#!/usr/bin/env bash
# Fast-forward the Jetson to this checkout's committed HEAD, without GitHub.
set -euo pipefail

if [[ ${1:-} == --help || $# -gt 2 ]]; then
  echo "Usage: $0 [ssh-host] [remote-repo]"
  echo "Defaults: jetson-usbc, ~/bv_ws/src/bv_core on the Jetson"
  echo "Transfers committed HEAD only; leaves local uncommitted edits alone."
  exit 0
fi

jetson_host=${1:-jetson-usbc}
remote_repo=${2:-bv_ws/src/bv_core}
script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
repo_dir=$(git -C "$script_dir" rev-parse --show-toplevel)
branch=$(git -C "$repo_dir" symbolic-ref --quiet --short HEAD) || {
  echo "Cannot sync a detached HEAD; check out a branch first." >&2
  exit 1
}
commit=$(git -C "$repo_dir" rev-parse HEAD)
sync_dir=$(mktemp -d)
trap 'rm -rf -- "$sync_dir"' EXIT
git -C "$repo_dir" bundle create "$sync_dir/repo.bundle" "refs/heads/$branch"

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

# SSH passes its command through the remote shell; quote every argument.
printf -v remote_command 'bash -c %q -- %q %q %q' \
  "$remote_script" "$remote_repo" "$branch" "$commit"
echo "Syncing $jetson_host:$remote_repo to $branch at ${commit:0:7}"
ssh "$jetson_host" "$remote_command" < "$sync_dir/repo.bundle"
