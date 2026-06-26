#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)

# Script is expected to run from publish/modules/src/xarm_ros2.
# Kept in src/xarm_ros2 and copied to publish by build script.
PUBLISH_DIR=$(cd "$SCRIPT_DIR/../../.." && pwd)
WS_DIR="$PUBLISH_DIR/modules"
INSTALL_DIR="$WS_DIR/install"
SDK_ROOT="$PUBLISH_DIR/xarm_can/package"
SDK_LIB_DIR="$SDK_ROOT/lib"
ROS_DISTRO_NAME="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/$ROS_DISTRO_NAME/setup.bash"
MODE="${1:-single}"

usage() {
  echo "Usage: $0 [single|bimanual] [extra_ros2_launch_args...]"
  echo "  single   : 启动单臂（默认）"
  echo "  bimanual : 启动双臂"
}

if [[ "$MODE" != "single" && "$MODE" != "bimanual" ]]; then
  if [[ "$MODE" == "-h" || "$MODE" == "--help" ]]; then
    usage
    exit 0
  fi
  echo "Invalid mode: $MODE"
  usage
  exit 1
fi

if [[ ! -f "$ROS_SETUP" ]]; then
  echo "ROS setup not found: $ROS_SETUP"
  exit 1
fi

if [[ ! -f "$INSTALL_DIR/setup.bash" ]]; then
  echo "publish install not found: $INSTALL_DIR/setup.bash"
  echo "Please build in publish/modules first:"
  echo "  cd $WS_DIR"
  echo "  source /opt/ros/$ROS_DISTRO_NAME/setup.bash"
  echo "  export XARM_SDK_ROOT=$PUBLISH_DIR/xarm_can/package"
  echo "  colcon build --base-paths src/xarm_ros2 src/xarm_description src/xarm_teleop --packages-select xarm xarm_bringup xarm_hardware xarm_description xarm_bimanual_moveit_config xarm_teleop --cmake-args -DXARM_SDK_ROOT=$PUBLISH_DIR/xarm_can/package"
  exit 1
fi

if [[ ! -f "$SDK_LIB_DIR/libxarm_can_sdk.so" ]]; then
  echo "SDK so not found: $SDK_LIB_DIR/libxarm_can_sdk.so"
  exit 1
fi

export LD_LIBRARY_PATH="$SDK_LIB_DIR:${LD_LIBRARY_PATH:-}"

if [[ ! -f "$SDK_LIB_DIR/libxarm_can_sdk.so.1" ]]; then
  ln -sf libxarm_can_sdk.so "$SDK_LIB_DIR/libxarm_can_sdk.so.1"
fi

set +u
source "$ROS_SETUP"
source "$INSTALL_DIR/setup.bash"
set -u

for pkg_prefix in \
  "$INSTALL_DIR/xarm_description" \
  "$INSTALL_DIR/xarm_hardware" \
  "$INSTALL_DIR/xarm_bringup"
do
  if [[ -d "$pkg_prefix" ]]; then
    export AMENT_PREFIX_PATH="$pkg_prefix:${AMENT_PREFIX_PATH:-}"
    export CMAKE_PREFIX_PATH="$pkg_prefix:${CMAKE_PREFIX_PATH:-}"
  fi
done

if [[ "$MODE" == "single" ]]; then
  shift || true
  exec ros2 launch xarm_bringup xarm.launch.py arm_type:=v10 "$@"
else
  shift || true
  exec ros2 launch xarm_bringup xarm.bimanual.launch.py arm_type:=v10 "$@"
fi
