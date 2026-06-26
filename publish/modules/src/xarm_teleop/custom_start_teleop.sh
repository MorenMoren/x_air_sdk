#!/usr/bin/env bash

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)

# 路径约定（与 start_xarm_ros2.sh 对齐）：
#   脚本位置: <publish>/modules/src/xarm_teleop/start_xarm_teleop.sh
#   publish : <publish>/
#   WS_DIR  : <publish>/modules/
#   install : <publish>/modules/install/
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

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)

cleanup() {
    echo ""
    echo "[INFO] 正在停止所有进程..."
    kill $PID1 $PID2 $PID3 2>/dev/null
    wait $PID1 $PID2 $PID3 2>/dev/null
    echo "[INFO] 已全部停止。"
    exit 0
}
trap cleanup SIGINT SIGTERM

echo "[INFO] 启动左臂遥操 1 (can1→can3)..."
"$SCRIPT_DIR/start_xarm_teleop.sh" unilateral_ros2 left_arm can1 can3 &
PID1=$!

echo "[INFO] 启动右臂遥操 2 (can2→can4)..."
"$SCRIPT_DIR/start_xarm_teleop.sh" unilateral_ros2 right_arm can2 can4 &
PID2=$!

echo "[INFO] 启动手柄桥接节点..."
ros2 launch xarm_teleop pico_bridge.launch.py &
PID3=$!

echo "=============================="
echo "  PID1 (遥操1):    $PID1"
echo "  PID2 (遥操2):    $PID2"
echo "  PID3 (pico桥接): $PID3"
echo "  Ctrl+C 停止所有..."
echo "=============================="

wait
