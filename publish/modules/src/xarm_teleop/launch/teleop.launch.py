
"""
teleop.launch.py  —  xarm_teleop 统一启动文件

支持四种运行模式：
  unilateral      单边遥操作（Leader → Follower）
  bilateral       双边力反馈遥操作
  gravity         重力补偿示教模式
  unilateral_ros2 单边遥操作 + ROS2 关节状态发布

用法（由 start_xarm_teleop.sh 调用）：
  ros2 launch xarm_teleop teleop.launch.py \\
      mode:=unilateral arm_side:=right_arm \\
      leader_can:=can0 follower_can:=can2 \\
      config_dir:=/path/to/config

URDF 由本 launch 文件在内部通过 xacro 自动生成，无需外部传入。
"""

import os
import shutil
import subprocess
import tempfile

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ─────────────────────────────────────────────────────────────────────────────
# 辅助：生成 URDF
# ─────────────────────────────────────────────────────────────────────────────
def _generate_urdf(context) -> tuple[str, str]:
    """使用 xacro 从 xarm_description 包生成 Leader / Follower URDF，返回两个路径。"""
    # 查找 v10.urdf.xacro
    xacro_path = ""
    candidates = []
    try:
        share_dir = get_package_share_directory("xarm_description")
        candidates.append(os.path.join(share_dir, "urdf", "robot", "v10.urdf.xacro"))
    except Exception:
        pass

    # 额外备用路径（源码树 / 发布树相对路径）
    this_dir = os.path.dirname(os.path.realpath(__file__))
    candidates += [
        os.path.join(this_dir, "..", "..", "xarm_description", "urdf", "robot", "v10.urdf.xacro"),
        os.path.join(this_dir, "..", "xarm_description", "urdf", "robot", "v10.urdf.xacro"),
    ]
    for c in candidates:
        if os.path.isfile(c):
            xacro_path = os.path.realpath(c)
            break

    if not xacro_path:
        raise RuntimeError(
            "未找到 v10.urdf.xacro。请确保 xarm_description 包已构建并在 AMENT_PREFIX_PATH 中。"
        )

    tmpdir = "/tmp/xarm_urdf_gen"
    os.makedirs(tmpdir, exist_ok=True)
    leader_urdf = os.path.join(tmpdir, "v10_leader.urdf")
    follower_urdf = os.path.join(tmpdir, "v10_follower.urdf")

    print(f"[teleop.launch] 生成 URDF: xacro {xacro_path} bimanual:=true")
    subprocess.run(
        ["xacro", xacro_path, "bimanual:=true", "-o", leader_urdf],
        check=True,
    )
    shutil.copy(leader_urdf, follower_urdf)
    return leader_urdf, follower_urdf


# ─────────────────────────────────────────────────────────────────────────────
# OpaqueFunction：根据 mode 参数决定启动哪个节点/进程
# ─────────────────────────────────────────────────────────────────────────────
def _launch_setup(context, *args, **kwargs):
    mode        = LaunchConfiguration("mode").perform(context)
    arm_side    = LaunchConfiguration("arm_side").perform(context)
    leader_can  = LaunchConfiguration("leader_can").perform(context)
    follower_can = LaunchConfiguration("follower_can").perform(context)
    config_dir  = LaunchConfiguration("config_dir").perform(context)

    # config_dir 为空时使用包内 config/ 目录
    if not config_dir:
        try:
            pkg_share = get_package_share_directory("xarm_teleop")
            config_dir = os.path.join(pkg_share, "config")
        except Exception:
            this_dir = os.path.dirname(os.path.realpath(__file__))
            config_dir = os.path.join(this_dir, "..", "config")
        config_dir = os.path.realpath(config_dir)

    leader_urdf, follower_urdf = _generate_urdf(context)

    # 找到 xarm_teleop 可执行文件目录
    pkg_lib_dir = os.path.join(
        get_package_prefix("xarm_teleop"), "lib", "xarm_teleop"
    )

    print(f"[teleop.launch] 模式={mode}  臂侧={arm_side}  "
          f"Leader={leader_can}  Follower={follower_can}")

    if mode == "unilateral_ros2":
        # ── ROS2 节点模式：通过 parameters 传参 ──────────────────────────────
        return [
            Node(
                package="xarm_teleop",
                executable="unilateral_control_ros2",
                name="xarm_teleop_node",
                output="screen",
                parameters=[{
                    "leader_urdf_path":   leader_urdf,
                    "follower_urdf_path": follower_urdf,
                    "arm_side":           arm_side,
                    "leader_can_if":      leader_can,
                    "follower_can_if":    follower_can,
                    "config_dir":         config_dir,
                }],
            ),
            
        ]

    elif mode == "gravity":
        # gravity_comp <arm_side> <can_if> <urdf_path> [config_dir]
        exe = os.path.join(pkg_lib_dir, "gravity_comp")
        return [ExecuteProcess(
            cmd=[exe, arm_side, leader_can, leader_urdf, config_dir],
            output="screen",
        )]

    elif mode == "bilateral":
        # bilateral_control <leader_urdf> <follower_urdf> <arm_side>
        #                   <leader_can> <follower_can> [config_dir]
        exe = os.path.join(pkg_lib_dir, "bilateral_control")
        return [ExecuteProcess(
            cmd=[exe, leader_urdf, follower_urdf,
                 arm_side, leader_can, follower_can, config_dir],
            output="screen",
        )]

    else:
        # unilateral (默认)
        # unilateral_control <leader_urdf> <follower_urdf> <arm_side>
        #                    <leader_can> <follower_can> [config_dir]
        exe = os.path.join(pkg_lib_dir, "unilateral_control")
        return [ExecuteProcess(
            cmd=[exe, leader_urdf, follower_urdf,
                 arm_side, leader_can, follower_can, config_dir],
            output="screen",
        )]


# ─────────────────────────────────────────────────────────────────────────────
# LaunchDescription
# ─────────────────────────────────────────────────────────────────────────────
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="unilateral",
            description="运行模式: unilateral | bilateral | gravity | unilateral_ros2",
        ),
        DeclareLaunchArgument(
            "arm_side",
            default_value="right_arm",
            description="机械臂侧别: right_arm | left_arm",
        ),
        DeclareLaunchArgument(
            "leader_can",
            default_value="can0",
            description="Leader 臂 CAN 接口，如 can0",
        ),
        DeclareLaunchArgument(
            "follower_can",
            default_value="can2",
            description="Follower 臂 CAN 接口，如 can2",
        ),
        DeclareLaunchArgument(
            "config_dir",
            default_value="",
            description="配置目录路径（留空使用包内默认 config/）",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
    pkg_share = get_package_share_directory('xarm_teleop')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'leader_urdf',
            default_value=os.path.join(pkg_share, 'urdf', 'leader_arm.urdf'),
            description='Leader arm URDF file'
        ),
        
        DeclareLaunchArgument(
            'follower_urdf',
            default_value=os.path.join(pkg_share, 'urdf', 'follower_arm.urdf'),
            description='Follower arm URDF file'
        ),
        
        DeclareLaunchArgument(
            'arm_type',
            default_value='right_arm',
            description='Arm type (right_arm or left_arm)'
        ),
        
        DeclareLaunchArgument(
            'leader_can',
            default_value='can0',
            description='Leader CAN interface'
        ),
        
        DeclareLaunchArgument(
            'follower_can',
            default_value='can2',
            description='Follower CAN interface'
        ),
        
        Node(
            package='xarm_teleop',
            executable='unilateral_control_ros2',
            name='bilateral_teleop',
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }],
            arguments=[
                LaunchConfiguration('leader_urdf'),
                LaunchConfiguration('follower_urdf'),
                LaunchConfiguration('arm_type'),
                LaunchConfiguration('leader_can'),
                LaunchConfiguration('follower_can'),
            ]
        ),
    ])