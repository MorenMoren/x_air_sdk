#!/usr/bin/env bash
set -euo pipefail

ARM_SIDE=${1:-right_arm}
LEADER_CAN_IF=${2:-}
FOLLOWER_CAN_IF=${3:-}
ARM_TYPE="v10"

if [[ "$ARM_SIDE" != "right_arm" && "$ARM_SIDE" != "left_arm" ]]; then
    echo "[ERROR] Invalid arm_side: $ARM_SIDE"
    echo "Usage: $0 <arm_side: right_arm|left_arm> [leader_can_if] [follower_can_if]"
    exit 1
fi

if [[ -z "$LEADER_CAN_IF" ]]; then
    LEADER_CAN_IF=$([[ "$ARM_SIDE" == "right_arm" ]] && echo "can0" || echo "can1")
fi
if [[ -z "$FOLLOWER_CAN_IF" ]]; then
    FOLLOWER_CAN_IF=$([[ "$ARM_SIDE" == "right_arm" ]] && echo "can2" || echo "can3")
fi

SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
PKG_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
WS_DIR=$(cd "$PKG_ROOT/../.." && pwd)
PKG_PARENT2=$(cd "$PKG_ROOT/../.." 2>/dev/null && pwd || true)
PREBUILT_CANDIDATE_A="$WS_DIR/prebuilt/xarm_teleop"
PREBUILT_CANDIDATE_B="$WS_DIR"
PREBUILT_CANDIDATE_C="$PKG_PARENT2"
MODULES_ROOT_CANDIDATE=$(cd "$WS_DIR/../.." 2>/dev/null && pwd || true)
PUBLISH_SDK_ROOT_A=$(cd "$PKG_ROOT/../../../xarm_can/package" 2>/dev/null && pwd || true)
PUBLISH_SDK_ROOT_B=$(cd "$MODULES_ROOT_CANDIDATE/../xarm_can/package" 2>/dev/null && pwd || true)
SOURCE_SDK_ROOT_A="$WS_DIR/xarm_can/sdk/package"
SOURCE_SDK_ROOT_B="$PKG_ROOT/../xarm_can/sdk/package"

ROS_SETUP="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
if [[ ! -f "$ROS_SETUP" ]]; then
    echo "[ERROR] ROS setup not found: $ROS_SETUP"
    exit 1
fi
set +u
# shellcheck source=/dev/null
source "$ROS_SETUP"
set -u
for setup in \
    "$WS_DIR/install/setup.bash" \
    "$MODULES_ROOT_CANDIDATE/install/setup.bash" \
    "$MODULES_ROOT_CANDIDATE/../install/setup.bash" \
    "$MODULES_ROOT_CANDIDATE/../../install/setup.bash"; do
    if [[ -f "$setup" ]]; then
        set +u
        # shellcheck source=/dev/null
        source "$setup"
        set -u
    fi
done

add_ld_path() {
    local p="$1"
    if [[ -d "$p" ]]; then
        export LD_LIBRARY_PATH="$p${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
    fi
}

add_sdk_root() {
    local root="$1"
    local lib_dir="$root/lib"
    local arch_lib_dir="$root/lib/$(uname -m)"
    # Check arch-specific subdir first (published SDK layout), then flat (source layout)
    if [[ -f "$arch_lib_dir/libxarm_can_sdk.so" ]]; then
        if [[ ! -f "$arch_lib_dir/libxarm_can_sdk.so.1" ]]; then
            ln -sf libxarm_can_sdk.so "$arch_lib_dir/libxarm_can_sdk.so.1"
        fi
        add_ld_path "$arch_lib_dir"
    elif [[ -f "$lib_dir/libxarm_can_sdk.so" ]]; then
        if [[ ! -f "$lib_dir/libxarm_can_sdk.so.1" ]]; then
            ln -sf libxarm_can_sdk.so "$lib_dir/libxarm_can_sdk.so.1"
        fi
        add_ld_path "$lib_dir"
    fi
}

add_ld_path "$WS_DIR/install/xarm_teleop/lib"
add_ld_path "$PREBUILT_CANDIDATE_A/lib"
add_ld_path "$PREBUILT_CANDIDATE_B/lib"
add_ld_path "$PREBUILT_CANDIDATE_C/lib"
add_sdk_root "$SOURCE_SDK_ROOT_A"
add_sdk_root "$SOURCE_SDK_ROOT_B"
if [[ -n "$PUBLISH_SDK_ROOT_A" ]]; then
    add_sdk_root "$PUBLISH_SDK_ROOT_A"
fi
if [[ -n "$PUBLISH_SDK_ROOT_B" ]]; then
    add_sdk_root "$PUBLISH_SDK_ROOT_B"
fi

CONFIG_DIR=""
for p in \
    "$WS_DIR/install/xarm_teleop/share/xarm_teleop/config" \
    "$PREBUILT_CANDIDATE_A/share/xarm_teleop/config" \
    "$PREBUILT_CANDIDATE_B/share/xarm_teleop/config" \
    "$PREBUILT_CANDIDATE_C/share/xarm_teleop/config" \
    "$PKG_ROOT/config"; do
    if [[ -d "$p" ]]; then
        CONFIG_DIR="$p"
        break
    fi
done
if [[ -z "$CONFIG_DIR" ]]; then
    echo "[ERROR] config directory not found."
    exit 1
fi

XACRO_PATH=""
for p in \
    "$WS_DIR/src/xarm_description/urdf/robot/${ARM_TYPE}.urdf.xacro" \
    "$WS_DIR/install/xarm_description/share/xarm_description/urdf/robot/${ARM_TYPE}.urdf.xacro" \
    "$MODULES_ROOT_CANDIDATE/src/xarm_description/urdf/robot/${ARM_TYPE}.urdf.xacro"; do
    if [[ -f "$p" ]]; then
        XACRO_PATH="$p"
        break
    fi
done
if [[ -z "$XACRO_PATH" ]]; then
    echo "[ERROR] xacro not found: ${ARM_TYPE}.urdf.xacro"
    exit 1
fi

if ! ros2 pkg executables xarm_teleop | grep -q unilateral_control_ros2; then
    echo "[ERROR] ROS2 executable not found: xarm_teleop/unilateral_control_ros2"
    echo "Please build xarm_teleop first."
    exit 1
fi

TMPDIR=$(mktemp -d /tmp/xarm_teleop_unilateral_ros2_XXXXXX)
trap 'rm -rf "$TMPDIR"' EXIT

LEADER_URDF_PATH="$TMPDIR/${ARM_TYPE}_leader.urdf"
FOLLOWER_URDF_PATH="$TMPDIR/${ARM_TYPE}_follower.urdf"

# Generate URDFs
echo "[INFO] Generating URDFs using xacro..."
xacro "$XACRO_PATH" bimanual:=true -o "$LEADER_URDF_PATH"
cp "$LEADER_URDF_PATH" "$FOLLOWER_URDF_PATH"

echo "[INFO] Launching unilateral control ROS2 node..."
ros2 run xarm_teleop unilateral_control_ros2 --ros-args \
    -p leader_urdf_path:="$LEADER_URDF_PATH" \
    -p follower_urdf_path:="$FOLLOWER_URDF_PATH" \
    -p arm_side:="$ARM_SIDE" \
    -p leader_can_if:="$LEADER_CAN_IF" \
    -p follower_can_if:="$FOLLOWER_CAN_IF" \
    -p config_dir:="$CONFIG_DIR"