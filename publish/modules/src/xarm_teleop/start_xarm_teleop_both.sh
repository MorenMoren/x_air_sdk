#!/usr/bin/env bash
# =============================================================================
# start_xarm_teleop_both.sh  —  同时启动双边遥操作（Right + Left）
#
# 与 start_xarm_teleop.sh 不同，本脚本在同一次执行中同时启动
# 右臂（can0→can2）和左臂（can1→can3）的遥操作。
#
# 使用方法:
#   ./start_xarm_teleop_both.sh [mode]
#
# mode        : unilateral | bilateral | unilateral_ros2（默认: unilateral）
#               gravity 模式不适用于双边启动，会被拒绝。
#
# CAN 接口固定分配:
#   右臂 Leader : can0, Follower: can2
#   左臂 Leader : can1, Follower: can3
#
# 启动前须先在 publish/modules 下编译（同 start_xarm_teleop.sh）。
#
# 示例:
#   ./start_xarm_teleop_both.sh unilateral
#   ./start_xarm_teleop_both.sh bilateral
#   ./start_xarm_teleop_both.sh unilateral_ros2
# =============================================================================
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)

# 路径约定（与 start_xarm_teleop.sh 对齐）
PUBLISH_DIR=$(cd "$SCRIPT_DIR/../../.." && pwd)
WS_DIR="$PUBLISH_DIR/modules"
INSTALL_DIR="$WS_DIR/install"
SDK_ROOT="$PUBLISH_DIR/xarm_can/package"
# Prefer arch-specific subdir (published SDK layout), fall back to flat (source layout)
if [[ -d "$SDK_ROOT/lib/$(uname -m)" ]]; then
    SDK_LIB_DIR="$SDK_ROOT/lib/$(uname -m)"
else
    SDK_LIB_DIR="$SDK_ROOT/lib"
fi
CONFIG_DIR="$SCRIPT_DIR/config"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/$ROS_DISTRO_NAME/setup.bash"

# ─── 参数解析 ────────────────────────────────────────────────────────────────
MODE="${1:-unilateral_ros2}"

usage() {
  cat <<EOF
用法: $0 [mode]

  mode  运行模式（默认: unilateral）
        unilateral      单边遥操作 × 2（左右臂同时运行）
        bilateral       双边力反馈遥操作 × 2（左右臂同时运行）
        unilateral_ros2 单边遥操作 + ROS2 关节状态发布 × 2

  gravity 模式不适合双边启动，请使用 start_xarm_teleop.sh gravity <side>

CAN 接口固定分配:
  右臂 Leader : can0  →  Follower: can2
  左臂 Leader : can1  →  Follower: can3

示例:
  $0 unilateral
  $0 bilateral
  $0 unilateral_ros2
EOF
}

if [[ "$MODE" == "-h" || "$MODE" == "--help" ]]; then usage; exit 0; fi
if [[ "$MODE" != "unilateral" && "$MODE" != "bilateral" && "$MODE" != "unilateral_ros2" ]]; then
  echo "Invalid mode: $MODE" >&2; usage; exit 1
fi

echo "========================================"
echo " xarm_teleop 双边启动 (Both Arms)"
echo "========================================"
echo "  模式        : $MODE"
echo "  右臂 CAN    : Leader=can0, Follower=can2"
echo "  左臂 CAN    : Leader=can1, Follower=can3"
echo "  工作空间    : $WS_DIR"
echo "========================================"

# ─── Step 1: source ROS2 基础环境 ──────────────────────────────────────────
if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS setup not found: $ROS_SETUP" >&2
  exit 1
fi
# shellcheck source=/dev/null
set +u; source "$ROS_SETUP"; set -u

# ─── Step 2: 检查 install，不负责编译 ────────────────────────────────────
if [[ ! -f "$INSTALL_DIR/setup.bash" ]]; then
  echo "publish install not found: $INSTALL_DIR/setup.bash" >&2
  echo "Please build in publish/modules first:" >&2
  echo "  cd $WS_DIR" >&2
  echo "  source /opt/ros/$ROS_DISTRO_NAME/setup.bash" >&2
  echo "  export XARM_SDK_ROOT=$SDK_ROOT" >&2
  echo "  colcon build \\" >&2
  echo "    --base-paths src/xarm_ros2 src/xarm_description src/multi_realsense \\" >&2
  echo "                 src/xarm_ik src/xarm_teleop \\" >&2
  echo "    --cmake-args -DXARM_SDK_ROOT=\$XARM_SDK_ROOT" >&2
  exit 1
fi

# ─── Step 3: source 工作空间 ────────────────────────────────────────────────
# shellcheck source=/dev/null
set +u; source "$INSTALL_DIR/setup.bash"; set -u

# ─── Step 4: 设置动态库路径 ─────────────────────────────────────────────────
if [[ ! -f "$SDK_LIB_DIR/libxarm_can_sdk.so" ]]; then
  echo "SDK so not found: $SDK_LIB_DIR/libxarm_can_sdk.so" >&2
  exit 1
fi

if [[ ! -f "$SDK_LIB_DIR/libxarm_can_sdk.so.1" ]]; then
  ln -sf libxarm_can_sdk.so "$SDK_LIB_DIR/libxarm_can_sdk.so.1"
fi

export LD_LIBRARY_PATH="$SDK_LIB_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

# ─── Step 5: 同时启动双边遥操作 ──────────────────────────────────────────────
# 左右臂各启动一个 ros2 launch，使用后台进程 + wait 保持脚本存活
# Ctrl+C 时 trap 会清理两个子进程

PIDS=()

cleanup() {
  echo ""
  echo "[INFO] 正在停止双边遥操作 ..."
  for pid in "${PIDS[@]}"; do
    kill "$pid" 3>/dev/null || true
  done
  wait 3>/dev/null || true
  echo "[INFO] 双边遥操作已停止。"
}
trap cleanup EXIT INT TERM

echo "[INFO] 启动右臂遥操作 (right_arm: can0 → can2) ..."
ros2 launch xarm_teleop teleop.launch.py \
  mode:="$MODE" \
  arm_side:="right_arm" \
  leader_can:="can1" \
  follower_can:="can3" \
  config_dir:="$CONFIG_DIR" &
PIDS+=($!)

echo "[INFO] 启动左臂遥操作 (left_arm: can1 → can3) ..."
ros2 launch xarm_teleop teleop.launch.py \
  mode:="$MODE" \
  arm_side:="left_arm" \
  leader_can:="can0" \
  follower_can:="can2" \
  config_dir:="$CONFIG_DIR" &
PIDS+=($!)

echo "[INFO] 启动手柄桥接节点..."
ros2 launch xarm_teleop pico_bridge.launch.py &
PIDS+=($!)

echo "[INFO] 双边遥操作已启动 (PIDs: ${PIDS[*]})，按 Ctrl+C 停止。"

# 等待所有子进程（Ctrl+C 触发 trap 清理，或任一进程退出后清理另一个）
wait

