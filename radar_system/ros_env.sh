#!/usr/bin/env bash
# Source this from each ROS entrypoint before changing its working directory.
robot_load_environment() {
  if [ "${_ROBOT_ENV_LOADED:-}" = 1 ]; then return 0; fi
  local dir exports setup candidate selected had_nounset=0
  dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)" || return 1
  if [ "${RK3588_RUNTIME_RESOLVED:-}" != 1 ] || [ -z "${RK3588_RUNTIME_ENV_SOURCE:-}" ] ||
      [ -z "${RK3588_ROS_ROOT:-}" ] || [ -z "${RK3588_PYTHON:-}" ] ||
      [ -z "${RK3588_POSE_MODEL:-}" ] || [ -z "${RK3588_ROBOT_CONFIG:-}" ] ||
      [ -z "${RK3588_DEPTH_PATH_CONFIG:-}" ]; then
    exports="$(/usr/bin/python3 "$dir/runtime_env_file.py")" || return 1
    eval "$exports"
  fi

  if [ -n "${ROS_DISTRO:-}" ]; then
    selected="$ROS_DISTRO"
    setup="$RK3588_ROS_ROOT/$selected/setup.bash"
    if [ ! -r "$setup" ]; then
      printf 'ROS_DISTRO=%s selected but setup is missing or unreadable: %s\n' "$selected" "$setup" >&2
      return 1
    fi
  else
    selected=''
    for candidate in humble jazzy; do
      setup="$RK3588_ROS_ROOT/$candidate/setup.bash"
      if [ -r "$setup" ]; then selected="$candidate"; break; fi
    done
    if [ -z "$selected" ]; then
      printf 'No supported ROS 2 setup found; checked %s and %s\n' \
        "$RK3588_ROS_ROOT/humble/setup.bash" "$RK3588_ROS_ROOT/jazzy/setup.bash" >&2
      return 1
    fi
  fi
  export ROS_DISTRO="$selected"
  case $- in *u*) had_nounset=1; set +u ;; esac
  if ! source "$setup"; then
    if [ "$had_nounset" = 1 ]; then set -u; fi
    printf 'ROS setup failed: %s\n' "$setup" >&2
    return 1
  fi
  if [ "$had_nounset" = 1 ]; then set -u; else set +u; fi
  if [ "${ROS_DISTRO:-}" != "$selected" ]; then
    printf 'ROS setup conflict: selected %s but %s set ROS_DISTRO=%s\n' \
      "$selected" "$setup" "${ROS_DISTRO:-<unset>}" >&2
    return 1
  fi
  # DDS 只在本机回环上做发现：车上的节点都在这台机器上，app 走的是网页服务的 WebSocket。
  # 否则节点对外报的是它启动那一刻的 WiFi IP，换了 WiFi（手机热点 ↔ 路由器）以后，之后再启动
  # 或崩溃重启的节点就找不到先前的节点（2026-09-25：网页服务重启后收不到底盘和 RTK）。
  # Humble 只认 ROS_LOCALHOST_ONLY（ROS_AUTOMATIC_DISCOVERY_RANGE 是 Iron 起才有的）；
  # ROS 的 env hook 每次 source setup.bash 都会把它改回 0，所以放在 source 之后。
  if [ "$ROS_DISTRO" = humble ]; then export ROS_LOCALHOST_ONLY=1; fi
  _ROBOT_ENV_LOADED=1
  export RK3588_RUNTIME_RESOLVED=1
  printf 'runtime: config=%s (%s) ROS=%s [%s] Python=%s model=%s robot_config=%s depth_config=%s\n' \
    "$ROBOT_RUNTIME_ENV" "$RK3588_RUNTIME_ENV_SOURCE" "$ROS_DISTRO" "$setup" \
    "$RK3588_PYTHON" "$RK3588_POSE_MODEL" "$RK3588_ROBOT_CONFIG" "$RK3588_DEPTH_PATH_CONFIG" >&2
}
robot_load_environment
