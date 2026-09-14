# Sourced by the sync_jetson*.sh scripts: picks the address the Jetson answers on.
# USB-C is tried first (faster link), then the Herelink network.

jetson_candidates=(bvorinnano@192.168.55.1 bvorinnano@192.168.144.2)

# True if sshd answers, even when this probe can't log in (password-only
# users, unknown host key); only connection failures count as unreachable.
jetson_reachable() {
  local err
  err=$(ssh -o ConnectTimeout=3 -o BatchMode=yes "$1" true 2>&1 >/dev/null) && return 0
  ! grep -qiE 'timed out|no route|refused|unreachable|could not resolve' <<<"$err"
}

# Prints the ssh target: the given override, else the first reachable candidate.
find_jetson() {
  if [[ -n ${1:-} ]]; then
    echo "$1"
    return
  fi
  local host
  for host in "${jetson_candidates[@]}"; do
    if jetson_reachable "$host"; then
      echo "$host"
      return
    fi
  done
  echo "Jetson not reachable at ${jetson_candidates[*]}; pass an ssh host to override." >&2
  return 1
}
