#!/usr/bin/env python3
# xarm_can SDK Python 基础功能测试 Demo
#
# 运行方式:
#   python3 sdk_python_basic_test.py --sdk ./sdk/package/lib/libxarm_can_sdk.so
#   python3 sdk_python_basic_test.py --can can1 --seconds 12 --amp 0.20
#
# 硬件 ID 映射说明（示例值，真机必须按实际修改）:
#   关节电机: send 0x01-0x07, recv 0x11-0x17 (共 7 个)
#   夹爪电机: send 0x08,      recv 0x18       (共 1 个)
#
# 测试流程:
#   1. 创建句柄、初始化关节 + 夹爪电机
#   2. 使能所有电机（关节 + 夹爪）
#   3. 基础通信验证：刷新 + 接收，打印关节和夹爪状态
#   4. 关节正弦运动演示（7 个关节，低频 0.12Hz）
#   5. 夹爪 MIT 控制演示（3s，位置正弦振荡）
#   6. 夹爪开合控制演示（open → close）
#   7. 所有电机回零（关节 + 夹爪 MIT 回 pos=0）
#   8. 安全失能 + 销毁句柄

import argparse
import ctypes
import math
import time
from ctypes import POINTER, c_char_p, c_float, c_int, c_uint32, c_void_p


class MitParam(ctypes.Structure):
    _fields_ = [
        ("pos",    c_float),
        ("vel",    c_float),
        ("kp",     c_float),
        ("kd",     c_float),
        ("torque", c_float),
    ]


class JointState(ctypes.Structure):
    """xarm_sdk_joint_state_t: 与 C 端内存布局一致"""
    _fields_ = [
        ("pos",    c_float),
        ("vel",    c_float),
        ("torque", c_float),
    ]


def bind_api(sdk):
    sdk.xarm_sdk_get_version.restype = c_char_p

    sdk.xarm_sdk_create.argtypes = [c_char_p, c_int, POINTER(c_void_p)]
    sdk.xarm_sdk_create.restype = c_int

    sdk.xarm_sdk_destroy.argtypes = [c_void_p]
    sdk.xarm_sdk_destroy.restype = c_int

    sdk.xarm_sdk_init_arm_motors.argtypes = [
        c_void_p, POINTER(c_int), POINTER(c_uint32), POINTER(c_uint32), c_int
    ]
    sdk.xarm_sdk_init_arm_motors.restype = c_int

    sdk.xarm_sdk_init_gripper_motor.argtypes = [c_void_p, c_int, c_uint32, c_uint32]
    sdk.xarm_sdk_init_gripper_motor.restype = c_int

    sdk.xarm_sdk_enable_all.argtypes = [c_void_p]
    sdk.xarm_sdk_enable_all.restype = c_int

    sdk.xarm_sdk_disable_all.argtypes = [c_void_p]
    sdk.xarm_sdk_disable_all.restype = c_int

    sdk.xarm_sdk_set_zero_all.argtypes = [c_void_p]
    sdk.xarm_sdk_set_zero_all.restype = c_int

    sdk.xarm_sdk_refresh_all.argtypes = [c_void_p]
    sdk.xarm_sdk_refresh_all.restype = c_int

    sdk.xarm_sdk_recv_all.argtypes = [c_void_p, c_int]
    sdk.xarm_sdk_recv_all.restype = c_int

    sdk.xarm_sdk_arm_mit_control.argtypes = [c_void_p, POINTER(MitParam), c_int]
    sdk.xarm_sdk_arm_mit_control.restype = c_int

    sdk.xarm_sdk_gripper_mit_control.argtypes = [c_void_p, POINTER(MitParam)]
    sdk.xarm_sdk_gripper_mit_control.restype = c_int

    sdk.xarm_sdk_gripper_open.argtypes = [c_void_p, c_float, c_float]
    sdk.xarm_sdk_gripper_open.restype = c_int

    sdk.xarm_sdk_gripper_close.argtypes = [c_void_p, c_float, c_float]
    sdk.xarm_sdk_gripper_close.restype = c_int

    sdk.xarm_sdk_set_callback_mode_state_all.argtypes = [c_void_p]
    sdk.xarm_sdk_set_callback_mode_state_all.restype = c_int

    sdk.xarm_sdk_get_arm_joint_states.argtypes = [c_void_p, POINTER(JointState), c_int]
    sdk.xarm_sdk_get_arm_joint_states.restype = c_int

    sdk.xarm_sdk_get_gripper_state.argtypes = [c_void_p, POINTER(JointState)]
    sdk.xarm_sdk_get_gripper_state.restype = c_int

    sdk.xarm_sdk_get_last_error.argtypes = [ctypes.c_char_p, c_int]
    sdk.xarm_sdk_get_last_error.restype = c_int


def get_last_error(sdk) -> str:
    buf = ctypes.create_string_buffer(512)
    sdk.xarm_sdk_get_last_error(buf, len(buf))
    return buf.value.decode("utf-8", errors="replace")


def check_ok(sdk, ret: int, step: str):
    if ret == 0:
        return
    raise RuntimeError(f"{step} 失败 ret={ret} detail={get_last_error(sdk)}")


def print_arm_states(sdk, handle, states_arr, joint_count: int, tag: str, in_motion: bool = False):
    """打印所有关节状态；in_motion=True 时若全为 0 输出诊断警告"""
    ret = sdk.xarm_sdk_get_arm_joint_states(handle, states_arr, joint_count)
    if ret != 0:
        print(f"[关节状态读取失败] {tag} ret={ret} err={get_last_error(sdk)}", flush=True)
        return
    info = "  ".join(
        f"J{j+1}[p={states_arr[j].pos:.3f} v={states_arr[j].vel:.3f} t={states_arr[j].torque:.3f}]"
        for j in range(joint_count)
    )
    print(f"[关节/{tag}] {info}", flush=True)
    if in_motion and all(
        states_arr[j].pos == 0.0 and states_arr[j].vel == 0.0 and states_arr[j].torque == 0.0
        for j in range(joint_count)
    ):
        print(
            "[诊断警告] 运动阶段所有关节状态均严格为 0！可能原因:\n"
            "  1. recv_ids 配置错误（电机实际响应 ID 与 recv_ids 不匹配）\n"
            "  2. CAN 接口或波特率不匹配\n"
            "  3. enable_fd 参数与电机配置不符\n"
            "  建议: 对照 examples/demo.cpp 确认 recv_can_ids 配置",
            flush=True,
        )


def print_gripper_state(sdk, handle, gs, tag: str):
    """打印夹爪状态"""
    ret = sdk.xarm_sdk_get_gripper_state(handle, ctypes.byref(gs))
    if ret != 0:
        print(f"[夹爪状态读取失败] {tag} ret={ret} err={get_last_error(sdk)}", flush=True)
        return
    print(f"[夹爪/{tag}] pos={gs.pos:.3f} vel={gs.vel:.3f} torque={gs.torque:.3f}", flush=True)


def parse_args():
    parser = argparse.ArgumentParser(description="xarm_can_sdk Python 基础功能测试")
    parser.add_argument("--sdk", default="./sdk/package/lib/libxarm_can_sdk.so")
    parser.add_argument("--can", default="can0")
    parser.add_argument("--fd", action="store_true")
    parser.add_argument("--seconds", type=int, default=12, help="关节正弦演示时长(秒)")
    parser.add_argument("--amp", type=float, default=0.20, help="关节正弦幅值(弧度)")
    return parser.parse_args()


def main():
    args = parse_args()
    sdk = ctypes.CDLL(args.sdk)
    bind_api(sdk)

    print("xarm_can_sdk version:", sdk.xarm_sdk_get_version().decode())
    print(f"can={args.can}, fd={int(args.fd)}, seconds={args.seconds}, amp={args.amp}")

    # ===== 硬件 ID 映射（示例值，真机使用前必须改成实际配置）=====
    # motor_types: 0=DM4310  1=DM4340  2=DM6006  3=DM8006  4=DM8009
    arm_count    = 7
    arm_types    = (c_int    * arm_count)(1, 1, 1, 1, 1, 1, 1)
    arm_send_ids = (c_uint32 * arm_count)(0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07)
    arm_recv_ids = (c_uint32 * arm_count)(0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17)
    gripper_motor_type = 1      # DM4340
    gripper_send_id    = 0x08
    gripper_recv_id    = 0x18
    # ============================================================

    handle = c_void_p()
    rc = 0

    try:
        # 1) 创建句柄
        check_ok(sdk,
            sdk.xarm_sdk_create(args.can.encode("utf-8"), 1 if args.fd else 0, ctypes.byref(handle)),
            "xarm_sdk_create")

        # 2) 初始化关节电机（7 个，0x01-0x07 / 0x11-0x17）
        check_ok(sdk,
            sdk.xarm_sdk_init_arm_motors(handle, arm_types, arm_send_ids, arm_recv_ids, arm_count),
            "xarm_sdk_init_arm_motors")

        # 3) 初始化夹爪电机（0x08 / 0x18）
        check_ok(sdk,
            sdk.xarm_sdk_init_gripper_motor(handle, gripper_motor_type,
                                            gripper_send_id, gripper_recv_id),
            "xarm_sdk_init_gripper_motor")

        # 4) 使能所有电机（关节 + 夹爪）
        check_ok(sdk, sdk.xarm_sdk_enable_all(handle), "xarm_sdk_enable_all")

        # 4.1) 使能后等待 2 秒接收使能响应帧
        print("使能成功，等待 2 秒接收使能响应...")
        for _ in range(20):
            sdk.xarm_sdk_recv_all(handle, 2000)
            time.sleep(0.1)

        # 4.2) 切换所有电机回调模式为 STATE
        check_ok(sdk, sdk.xarm_sdk_set_callback_mode_state_all(handle),
                 "xarm_sdk_set_callback_mode_state_all")

        # 打印使能后初始状态（零位 p/v/t 接近0正常）
        arm_states = (JointState * arm_count)()
        gripper_state = JointState()
        print("使能后初始状态（零位 p/v/t 接近0正常）:")
        print_arm_states(sdk, handle, arm_states, arm_count, "初始")
        print_gripper_state(sdk, handle, gripper_state, "初始")

        # ========== 阶段 5：基础通信验证 ==========
        print("\n== 阶段5: 基础刷新验证（静止零位，p/v/t 接近0正常）==")
        for i in range(10):
            check_ok(sdk, sdk.xarm_sdk_refresh_all(handle), "xarm_sdk_refresh_all")
            sdk.xarm_sdk_recv_all(handle, 2000)
            time.sleep(0.1)
            label = f"刷新#{i+1}"
            print_arm_states(sdk, handle, arm_states, arm_count, label)
            print_gripper_state(sdk, handle, gripper_state, label)

        # ========== 阶段 6：关节正弦运动演示 ==========
        print(f"\n== 阶段6: 关节正弦运动 {args.seconds}s amp={args.amp}rad ==")
        ArmCmdArray = MitParam * arm_count
        t0 = time.time()
        last_print_sec = -1
        while True:
            t = time.time() - t0
            if t >= args.seconds:
                break
            cmds = ArmCmdArray()
            for i in range(arm_count):
                phase = 0.0 if i % 2 == 0 else math.pi
                cmds[i].pos    = args.amp * math.sin(2.0 * math.pi * 0.12 * t + phase)
                cmds[i].vel    = 0.0
                cmds[i].kp     = 8.0
                cmds[i].kd     = 0.8
                cmds[i].torque = 0.0
            check_ok(sdk, sdk.xarm_sdk_arm_mit_control(handle, cmds, arm_count),
                     "xarm_sdk_arm_mit_control")
            sdk.xarm_sdk_recv_all(handle, 500)
            sec_i = int(t)
            if sec_i != last_print_sec:
                last_print_sec = sec_i
                print_arm_states(sdk, handle, arm_states, arm_count,
                                 f"{sec_i}/{args.seconds}s 运动", in_motion=True)
            time.sleep(0.01)

        # ========== 阶段 7：夹爪 MIT 控制演示 ==========
        print("\n== 阶段7: 夹爪 MIT 控制演示 (3s) ==")
        GripperCmd = MitParam()
        t0 = time.time()
        last_print_sec = -1
        while True:
            t = time.time() - t0
            if t >= 3.0:
                break
            GripperCmd.pos    = 0.3 * math.sin(2.0 * math.pi * 0.5 * t)
            GripperCmd.vel    = 0.0
            GripperCmd.kp     = 10.0
            GripperCmd.kd     = 0.5
            GripperCmd.torque = 0.0
            check_ok(sdk, sdk.xarm_sdk_gripper_mit_control(handle, ctypes.byref(GripperCmd)),
                     "xarm_sdk_gripper_mit_control")
            sdk.xarm_sdk_recv_all(handle, 500)
            sec_i = int(t)
            if sec_i != last_print_sec:
                last_print_sec = sec_i
                print_gripper_state(sdk, handle, gripper_state, f"MIT {sec_i}s")
            time.sleep(0.01)

        # ========== 阶段 8：夹爪开合控制演示 ==========
        # open:  motor_pos = -1.0472 rad (约 -60°，物理张开)
        # close: motor_pos = 0.0 rad     (电机零位，物理闭合)
        print("\n== 阶段8: 夹爪开合控制演示 ==")

        print("  张开夹爪 (open, kp=50, kd=1)...")
        check_ok(sdk, sdk.xarm_sdk_gripper_open(handle, 50.0, 1.0), "xarm_sdk_gripper_open")
        for step in range(150):
            sdk.xarm_sdk_recv_all(handle, 500)
            if step % 50 == 0:
                print_gripper_state(sdk, handle, gripper_state, f"张开{step*10}ms")
            time.sleep(0.01)

        print("  闭合夹爪 (close, kp=50, kd=1)...")
        check_ok(sdk, sdk.xarm_sdk_gripper_close(handle, 50.0, 1.0), "xarm_sdk_gripper_close")
        for step in range(150):
            sdk.xarm_sdk_recv_all(handle, 500)
            if step % 50 == 0:
                print_gripper_state(sdk, handle, gripper_state, f"闭合{step*10}ms")
            time.sleep(0.01)

        # ========== 阶段 9：所有电机回零 ==========
        print("\n== 阶段9: 所有电机回零 (2s) ==")
        for step in range(200):
            # 7 个关节回零
            zero_cmds = ArmCmdArray()
            for i in range(arm_count):
                zero_cmds[i].pos    = 0.0
                zero_cmds[i].vel    = 0.0
                zero_cmds[i].kp     = 8.0
                zero_cmds[i].kd     = 0.8
                zero_cmds[i].torque = 0.0
            check_ok(sdk, sdk.xarm_sdk_arm_mit_control(handle, zero_cmds, arm_count),
                     "xarm_sdk_arm_mit_control(return_zero)")
            # 夹爪回零（motor_pos=0 即闭合位置）
            GripperCmd.pos    = 0.0
            GripperCmd.vel    = 0.0
            GripperCmd.kp     = 8.0
            GripperCmd.kd     = 0.8
            GripperCmd.torque = 0.0
            check_ok(sdk, sdk.xarm_sdk_gripper_mit_control(handle, ctypes.byref(GripperCmd)),
                     "xarm_sdk_gripper_mit_control(return_zero)")
            sdk.xarm_sdk_recv_all(handle, 500)
            if step % 50 == 0:
                print_arm_states(sdk, handle, arm_states, arm_count, f"回零{step*10}ms")
                print_gripper_state(sdk, handle, gripper_state, f"回零{step*10}ms")
            time.sleep(0.01)

    except Exception as exc:
        rc = 1
        print("[ERROR]", exc)
    finally:
        # 10) 安全失能 + 销毁
        print("\n== 阶段10: 失能所有电机 ==")
        if handle:
            sdk.xarm_sdk_disable_all(handle)
            sdk.xarm_sdk_recv_all(handle, 1000)
            sdk.xarm_sdk_destroy(handle)

    print("测试结束 rc=", rc)
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
