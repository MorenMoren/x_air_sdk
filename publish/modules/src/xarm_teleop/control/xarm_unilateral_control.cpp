#include <chrono>
#include <csignal>
#include <cstdio>
#include <thread>
#include <xarm_teleop_sdk.h>

static xarm_teleop_handle_t g_handle = nullptr;
static volatile std::sig_atomic_t g_stop_requested = 0;

static void on_signal(int /*sig*/) {
    g_stop_requested = 1;
}

static void print_usage(const char *prog) {
    fprintf(stderr,
            "用法: %s <leader_urdf> <follower_urdf> <arm_side> <leader_can> <follower_can> [config_dir]\n"
            "  arm_side    : right_arm | left_arm\n"
            "  leader_can  : Leader 臂 CAN 接口，如 can0\n"
            "  follower_can: Follower 臂 CAN 接口，如 can2\n"
            "  config_dir  : 配置目录（含 leader.yaml/follower.yaml），默认: config\n",
            prog);
}

int main(int argc, char **argv) {
    if (argc < 6) {
        print_usage(argv[0]);
        return 1;
    }

    const char *leader_urdf   = argv[1];
    const char *follower_urdf = argv[2];
    const char *arm_side      = argv[3];
    const char *leader_can    = argv[4];
    const char *follower_can  = argv[5];
    const char *config_dir    = (argc >= 7) ? argv[6] : "config";

    fprintf(stdout,
            "=== XArm 单边遥操作 (SDK %s) ===\n"
            "  arm_side      : %s\n"
            "  leader_can    : %s\n"
            "  follower_can  : %s\n"
            "  leader_urdf   : %s\n"
            "  follower_urdf : %s\n"
            "  config_dir    : %s\n",
            xarm_teleop_version(), arm_side, leader_can, follower_can,
            leader_urdf, follower_urdf, config_dir);

    int ret = xarm_teleop_create_unilateral(leader_can, follower_can,
                                             leader_urdf, follower_urdf,
                                             arm_side, config_dir, &g_handle);
    if (ret != XARM_TELEOP_OK) {
        fprintf(stderr, "[ERROR] 初始化失败: %s\n", xarm_teleop_get_last_error());
        return 1;
    }

    signal(SIGINT, on_signal);
    signal(SIGTERM, on_signal);

    ret = xarm_teleop_start(g_handle);
    if (ret != XARM_TELEOP_OK) {
        fprintf(stderr, "[ERROR] 启动失败: %s\n", xarm_teleop_get_last_error());
        xarm_teleop_destroy(g_handle);
        return 1;
    }

    fprintf(stdout, "控制循环运行中，按 Ctrl+C 停止...\n");
    while (!g_stop_requested && xarm_teleop_is_running(g_handle) == 1) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (xarm_teleop_is_running(g_handle) == 1) {
        xarm_teleop_stop(g_handle);
    }
    xarm_teleop_destroy(g_handle);
    g_handle = nullptr;
    fprintf(stdout, "已停止。\n");
    return 0;
}
