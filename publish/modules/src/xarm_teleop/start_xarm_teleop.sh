#!/usr/bin/env bash
# =============================================================================
# start_xarm_teleop.sh  —  xarm_teleop 启动脚本（只运行，不编译）
#
# 脚本必须在 publish/modules/src/xarm_teleop/ 目录下执行（与 start_xarm_ros2.sh 结构一致）。
#
# 使用方法:
#   ./start_xarm_teleop.sh [mode] [arm_side] [leader_can] [follower_can]
#
# mode        : unilateral | bilateral | gravity | unilateral_ros2（默认: unilateral）
# arm_side    : right_arm | left_arm（默认: right_arm）
# leader_can  : Leader 臂 CAN 接口（默认: right→can0, left→can1）
# follower_can: Follower 臂 CAN 接口（默认: right→can2, left→can3）
#
# 启动前须先在 publish/modules 下编译：
#   cd <publish>/modules
#   source /opt/ros/humble/setup.bash
#   export XARM_SDK_ROOT=<publish>/xarm_can/package
#   colcon build \
#     --base-paths src/xarm_ros2 src/xarm_description src/multi_realsense \
#                  src/xarm_ik src/xarm_teleop \
#     --cmake-args -DXARM_SDK_ROOT=$XARM_SDK_ROOT
#
# 示例:
#   ./start_xarm_teleop.sh unilateral right_arm can0 can2
#   ./start_xarm_teleop.sh bilateral left_arm can1 can3
#   ./start_xarm_teleop.sh gravity right_arm can0
#   ./start_xarm_teleop.sh unilateral_ros2 right_arm can0 can2
# =============================================================================
set -euo pipefail

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

# ─── 参数解析 ────────────────────────────────────────────────────────────────
MODE="${1:-unilateral}"
ARM_SIDE="${2:-right_arm}"
LEADER_CAN="${3:-}"
FOLLOWER_CAN="${4:-}"

usage() {
  cat <<EOF
用法: $0 [mode] [arm_side] [leader_can] [follower_can]

  mode        运行模式（默认: unilateral）
              unilateral      单边遥操作（Leader → Follower）
              bilateral       双边力反馈遥操作
              gravity         重力补偿示教模式（仅使用 leader_can）
              unilateral_ros2 单边遥操作 + ROS2 关节状态发布

  arm_side    机械臂侧别（默认: right_arm）
              right_arm | left_arm

  leader_can  Leader 臂 CAN 接口（默认: right→can0, left→can1）
  follower_can Follower 臂 CAN 接口（默认: right→can2, left→can3）

启动前须先编译（在 publish/modules 下执行一次）：
  cd $WS_DIR
  source /opt/ros/$ROS_DISTRO_NAME/setup.bash
  export XARM_SDK_ROOT=$SDK_ROOT
  colcon build \\
    --base-paths src/xarm_ros2 src/xarm_description src/multi_realsense \\
                 src/xarm_ik src/xarm_teleop \\
    --cmake-args -DXARM_SDK_ROOT=\$XARM_SDK_ROOT

示例:
  $0 unilateral right_arm can0 can2
  $0 bilateral left_arm can1 can3
  $0 gravity right_arm can0
  $0 unilateral_ros2 right_arm can0 can2
EOF
}

if [[ "$MODE" == "-h" || "$MODE" == "--help" ]]; then usage; exit 0; fi
if [[ "$MODE" != "unilateral" && "$MODE" != "bilateral" && \
      "$MODE" != "gravity" && "$MODE" != "unilateral_ros2" ]]; then
  echo "Invalid mode: $MODE" >&2; usage; exit 1
fi
if [[ "$ARM_SIDE" != "right_arm" && "$ARM_SIDE" != "left_arm" ]]; then
  echo "Invalid arm_side: $ARM_SIDE" >&2; usage; exit 1
fi

if [[ -z "$LEADER_CAN" ]]; then
  LEADER_CAN=$([[ "$ARM_SIDE" == "right_arm" ]] && echo "can0" || echo "can1")
fi
if [[ -z "$FOLLOWER_CAN" ]]; then
  FOLLOWER_CAN=$([[ "$ARM_SIDE" == "right_arm" ]] && echo "can2" || echo "can3")
fi

echo "========================================"
echo " xarm_teleop 启动"
echo "========================================"
echo "  模式        : $MODE"
echo "  机械臂侧    : $ARM_SIDE"
echo "  Leader CAN  : $LEADER_CAN"
echo "  Follower CAN: $FOLLOWER_CAN"
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

# ─── Step 5: ros2 launch ────────────────────────────────────────────────────
# URDF 由 teleop.launch.py 内部通过 xacro 自动生成
echo "[INFO] 启动 xarm_teleop ..."
exec ros2 launch xarm_teleop teleop.launch.py \
  mode:="$MODE" \
  arm_side:="$ARM_SIDE" \
  leader_can:="$LEADER_CAN" \
  follower_can:="$FOLLOWER_CAN" \
  config_dir:="$CONFIG_DIR"
