// xarm_can SDK C++ 基础功能测试 Demo
//
// 编译方式 (推荐使用 sdk/scripts/build_demo.sh):
//   bash sdk/scripts/build_demo.sh /tmp
//   /tmp/sdk_cpp_basic_test can0 0 12 0.20
//
// 硬件 ID 映射说明（示例值，真机必须按实际修改）:
//   关节电机: send 0x01-0x07, recv 0x11-0x17 (共 7 个)
//   夹爪电机: send 0x08,      recv 0x18       (共 1 个)
//
// 测试流程:
//   1. 创建句柄、初始化关节 + 夹爪电机
//   2. 使能所有电机（关节 + 夹爪）
//   3. 基础通信验证：刷新 + 接收，打印关节和夹爪状态
//   4. 关节正弦运动演示（7 个关节，低频 0.12Hz，可视化运动幅度）
//   5. 夹爪 MIT 控制演示（3s，位置正弦振荡，验证 MIT 通路）
//   6. 夹爪开合控制演示（open → close → 回中）
//   7. 所有电机回零（关节 + 夹爪 MIT 回 pos=0）
//   8. 安全失能 + 销毁句柄

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <vector>
#include <thread>

#include "xarm_can_sdk.h"

namespace {

// ========== 辅助工具 ==========

bool sdk_ok(int ret, const char* step) {
    if (ret == XARM_SDK_OK) return true;
    char err[512] = {0};
    xarm_sdk_get_last_error(err, static_cast<int>(sizeof(err)));
    std::cerr << "[FAIL] " << step << " ret=" << ret << " detail=" << err << "\n";
    return false;
}

// 打印所有关节状态
// in_motion=true 时若全零则输出诊断警告（运动期间 pos 应出现非零值）
void print_arm_states(xarm_sdk_handle_t h, int joint_count,
                      const char* tag, bool in_motion = false) {
    std::vector<xarm_sdk_joint_state_t> states(static_cast<size_t>(joint_count));
    int ret = xarm_sdk_get_arm_joint_states(h, states.data(), joint_count);
    if (ret != XARM_SDK_OK) {
        char err[256] = {0};
        xarm_sdk_get_last_error(err, sizeof(err));
        std::cerr << "[关节状态读取失败] tag=" << tag
                  << " ret=" << ret << " err=" << err << "\n";
        return;
    }
    bool all_zero = true;
    std::cout << "[关节/" << tag << "]";
    for (int j = 0; j < joint_count; ++j) {
        std::cout << " J" << (j + 1)
                  << "[p=" << states[j].pos
                  << " v=" << states[j].vel
                  << " t=" << states[j].torque << "]";
        if (states[j].pos != 0.0f || states[j].vel != 0.0f || states[j].torque != 0.0f)
            all_zero = false;
    }
    std::cout << "\n";
    if (in_motion && all_zero) {
        std::cerr << "[诊断警告] 运动阶段所有关节状态均严格为 0！可能原因:\n"
                  << "  1. recv_ids 配置错误（电机实际响应 ID 与 recv_ids 数组不匹配）\n"
                  << "  2. CAN 接口或波特率不匹配（请确认 can_if 参数与电机所在总线一致）\n"
                  << "  3. enable_fd 参数与电机固件配置不符（CAN-FD vs 标准 CAN）\n"
                  << "  建议：对照 examples/demo.cpp 确认 recv_can_ids 配置\n";
    }
}

// 打印夹爪状态
void print_gripper_state(xarm_sdk_handle_t h, const char* tag) {
    xarm_sdk_joint_state_t gs{};
    int ret = xarm_sdk_get_gripper_state(h, &gs);
    if (ret != XARM_SDK_OK) {
        char err[256] = {0};
        xarm_sdk_get_last_error(err, sizeof(err));
        std::cerr << "[夹爪状态读取失败] tag=" << tag
                  << " ret=" << ret << " err=" << err << "\n";
        return;
    }
    std::cout << "[夹爪/" << tag << "] pos=" << gs.pos
              << " vel=" << gs.vel << " torque=" << gs.torque << "\n";
}

void print_usage(const char* prog) {
    std::cout << "用法: " << prog << " [can_if] [enable_fd:0|1] [demo_seconds] [amp_rad]\n"
              << "示例: " << prog << " can0 0 12 0.20\n"
              << "说明: demo_seconds=关节正弦演示时长(秒), amp_rad=关节正弦幅值(弧度)\n";
}

}  // namespace

int main(int argc, char** argv) {
    if (argc > 1 && std::strcmp(argv[1], "--help") == 0) {
        print_usage(argv[0]);
        return 0;
    }

    const char* can_if    = (argc > 1) ? argv[1] : "can0";
    const int   enable_fd = (argc > 2) ? std::atoi(argv[2]) : 0;
    const int   demo_secs = (argc > 3) ? std::atoi(argv[3]) : 12;
    const float amp_rad   = (argc > 4) ? static_cast<float>(std::atof(argv[4])) : 0.20f;

    std::cout << "xarm_can_sdk version: " << xarm_sdk_get_version() << "\n"
              << "can_if=" << can_if << " enable_fd=" << enable_fd
              << " demo_secs=" << demo_secs << " amp_rad=" << amp_rad << "\n";

    // ===== 硬件 ID 映射（示例值，真机使用前必须改成实际配置）=====
    // motor_types: 0=DM4310  1=DM4340  2=DM6006  3=DM8006  4=DM8009
    constexpr int kArmCount = 7;
    int      arm_motor_types[kArmCount] = {1, 1, 1, 1, 1, 1, 1};
    uint32_t arm_send_ids[kArmCount]    = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
    uint32_t arm_recv_ids[kArmCount]    = {0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17};
    constexpr int      kGripperMotorType = 1;    // DM4340
    constexpr uint32_t kGripperSendId    = 0x08;
    constexpr uint32_t kGripperRecvId    = 0x18;
    // =============================================================

    xarm_sdk_handle_t h = nullptr;

    // 1) 创建句柄（建立与 CAN 总线的连接上下文）
    if (!sdk_ok(xarm_sdk_create(can_if, enable_fd, &h), "xarm_sdk_create")) {
        return 1;
    }

    int rc = 0;
    do {
        // 2) 初始化关节电机（7 个，0x01-0x07 发送，0x11-0x17 接收）
        if (!sdk_ok(xarm_sdk_init_arm_motors(h, arm_motor_types,
                                             arm_send_ids, arm_recv_ids, kArmCount),
                    "xarm_sdk_init_arm_motors")) {
            rc = 2; break;
        }

        // 3) 初始化夹爪电机（0x08 发送，0x18 接收）
        if (!sdk_ok(xarm_sdk_init_gripper_motor(h, kGripperMotorType,
                                                kGripperSendId, kGripperRecvId),
                    "xarm_sdk_init_gripper_motor")) {
            rc = 3; break;
        }

        // 4) 使能所有电机（关节 + 夹爪一起使能）
        if (!sdk_ok(xarm_sdk_enable_all(h), "xarm_sdk_enable_all")) {
            rc = 4; break;
        }

        // 4.1) 使能后等待 2 秒，接收所有电机的使能响应帧
        std::cout << "使能成功，等待 2 秒接收使能响应...\n";
        for (int i = 0; i < 20; ++i) {
            xarm_sdk_recv_all(h, 2000);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // 4.2) 切换所有电机回调模式为 STATE，使 recv_all 解析并更新电机状态
        if (!sdk_ok(xarm_sdk_set_callback_mode_state_all(h),
                    "xarm_sdk_set_callback_mode_state_all")) {
            rc = 4; break;
        }

        // 打印使能后初始状态（零位时 p/v/t 均接近 0，属正常）
        std::cout << "使能后初始状态（零位 p/v/t 接近0属正常）:\n";
        print_arm_states(h, kArmCount, "初始");
        print_gripper_state(h, "初始");

        // ========== 阶段 5：基础通信验证 ==========
        std::cout << "\n== 阶段5: 基础刷新验证（静止零位，p/v/t 接近0正常）==\n";
        for (int i = 0; i < 10; ++i) {
            if (!sdk_ok(xarm_sdk_refresh_all(h), "xarm_sdk_refresh_all")) {
                rc = 5; break;
            }
            xarm_sdk_recv_all(h, 2000);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

            char label[32];
            std::snprintf(label, sizeof(label), "刷新#%d", i + 1);
            print_arm_states(h, kArmCount, label);
            print_gripper_state(h, label);
        }
        if (rc != 0) break;

        // ========== 阶段 6：关节正弦运动演示 ==========
        std::cout << "\n== 阶段6: 关节正弦运动 " << demo_secs << "s amp=" << amp_rad << "rad ==\n";
        {
            constexpr double kPi = 3.14159265358979323846;
            constexpr double kFreqHz = 0.12;
            const auto kStep = std::chrono::milliseconds(10);  // 100Hz
            auto t0 = std::chrono::steady_clock::now();
            int last_print_sec = -1;

            while (true) {
                double t = std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - t0).count();
                if (t >= demo_secs) break;

                xarm_sdk_mit_param_t cmd[kArmCount] = {};
                for (int i = 0; i < kArmCount; ++i) {
                    const double phase = (i % 2 == 0) ? 0.0 : kPi;
                    cmd[i].pos    = static_cast<float>(amp_rad * std::sin(2.0 * kPi * kFreqHz * t + phase));
                    cmd[i].vel    = 0.0f;
                    cmd[i].kp     = 8.0f;
                    cmd[i].kd     = 0.8f;
                    cmd[i].torque = 0.0f;
                }
                if (!sdk_ok(xarm_sdk_arm_mit_control(h, cmd, kArmCount),
                            "xarm_sdk_arm_mit_control")) {
                    rc = 6; break;
                }
                xarm_sdk_recv_all(h, 500);

                const int sec_i = static_cast<int>(t);
                if (sec_i != last_print_sec) {
                    last_print_sec = sec_i;
                    char label[32];
                    std::snprintf(label, sizeof(label), "%ds/%ds 运动", sec_i, demo_secs);
                    print_arm_states(h, kArmCount, label, /*in_motion=*/true);
                }
                std::this_thread::sleep_for(kStep);
            }
        }
        if (rc != 0) break;

        // ========== 阶段 7：夹爪 MIT 控制演示 ==========
        // 直接下发 MIT 命令让夹爪电机在 ±0.3rad 之间振荡 3 秒
        std::cout << "\n== 阶段7: 夹爪 MIT 控制演示 (3s) ==\n";
        {
            constexpr double kPi = 3.14159265358979323846;
            auto t0 = std::chrono::steady_clock::now();
            int last_print_sec = -1;

            while (true) {
                double t = std::chrono::duration<double>(
                               std::chrono::steady_clock::now() - t0).count();
                if (t >= 3.0) break;

                xarm_sdk_mit_param_t gcmd{};
                gcmd.pos    = static_cast<float>(0.3 * std::sin(2.0 * kPi * 0.5 * t));
                gcmd.vel    = 0.0f;
                gcmd.kp     = 10.0f;
                gcmd.kd     = 0.5f;
                gcmd.torque = 0.0f;
                if (!sdk_ok(xarm_sdk_gripper_mit_control(h, &gcmd),
                            "xarm_sdk_gripper_mit_control")) {
                    rc = 7; break;
                }
                xarm_sdk_recv_all(h, 500);

                const int sec_i = static_cast<int>(t);
                if (sec_i != last_print_sec) {
                    last_print_sec = sec_i;
                    char label[32];
                    std::snprintf(label, sizeof(label), "MIT %ds", sec_i);
                    print_gripper_state(h, label);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        if (rc != 0) break;

        // ========== 阶段 8：夹爪开合控制演示 ==========
        // 调用高层 open/close 接口（内部映射到 MIT 控制特定角度）
        // open:  motor_pos = -1.0472 rad (约 -60°，物理张开)
        // close: motor_pos = 0.0 rad     (电机零位，物理闭合)
        std::cout << "\n== 阶段8: 夹爪开合控制演示 ==\n";

        // 8.1) 张开夹爪
        std::cout << "  张开夹爪 (open, kp=50, kd=1)...\n";
        if (!sdk_ok(xarm_sdk_gripper_open(h, 50.0f, 1.0f), "xarm_sdk_gripper_open")) {
            rc = 8; break;
        }
        for (int i = 0; i < 150; ++i) {
            xarm_sdk_recv_all(h, 500);
            if (i % 50 == 0) {
                char label[32];
                std::snprintf(label, sizeof(label), "张开%dms", i * 10);
                print_gripper_state(h, label);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // 8.2) 闭合夹爪
        std::cout << "  闭合夹爪 (close, kp=50, kd=1)...\n";
        if (!sdk_ok(xarm_sdk_gripper_close(h, 50.0f, 1.0f), "xarm_sdk_gripper_close")) {
            rc = 8; break;
        }
        for (int i = 0; i < 150; ++i) {
            xarm_sdk_recv_all(h, 500);
            if (i % 50 == 0) {
                char label[32];
                std::snprintf(label, sizeof(label), "闭合%dms", i * 10);
                print_gripper_state(h, label);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // ========== 阶段 9：所有电机回零 ==========
        std::cout << "\n== 阶段9: 所有电机回零 (2s) ==\n";
        for (int j = 0; j < 200; ++j) {
            // 7 个关节回零
            xarm_sdk_mit_param_t zero_cmd[kArmCount] = {};
            for (int i = 0; i < kArmCount; ++i) {
                zero_cmd[i].pos    = 0.0f;
                zero_cmd[i].vel    = 0.0f;
                zero_cmd[i].kp     = 8.0f;
                zero_cmd[i].kd     = 0.8f;
                zero_cmd[i].torque = 0.0f;
            }
            if (!sdk_ok(xarm_sdk_arm_mit_control(h, zero_cmd, kArmCount),
                        "xarm_sdk_arm_mit_control(return_zero)")) {
                rc = 9; break;
            }
            // 夹爪回零（motor_pos=0 即闭合位置）
            xarm_sdk_mit_param_t gripper_zero{};
            gripper_zero.pos    = 0.0f;
            gripper_zero.vel    = 0.0f;
            gripper_zero.kp     = 8.0f;
            gripper_zero.kd     = 0.8f;
            gripper_zero.torque = 0.0f;
            if (!sdk_ok(xarm_sdk_gripper_mit_control(h, &gripper_zero),
                        "xarm_sdk_gripper_mit_control(return_zero)")) {
                rc = 9; break;
            }
            xarm_sdk_recv_all(h, 500);
            if (j % 50 == 0) {
                char label[32];
                std::snprintf(label, sizeof(label), "回零%dms", j * 10);
                print_arm_states(h, kArmCount, label);
                print_gripper_state(h, label);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

    } while (false);

    // 10) 安全失能 + 销毁
    // 必须在 disable_all 后调用 recv_all，确保失能命令被电机接收
    std::cout << "\n== 阶段10: 失能所有电机 ==\n";
    sdk_ok(xarm_sdk_disable_all(h), "xarm_sdk_disable_all");
    xarm_sdk_recv_all(h, 1000);
    xarm_sdk_destroy(h);

    std::cout << "测试结束，rc=" << rc << std::endl;
    return rc;
}
