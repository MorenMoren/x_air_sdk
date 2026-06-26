// XArm Teleop SDK C API 实现
#include "xarm_teleop_sdk.h"

#include <atomic>
#include <chrono>
#include <filesystem>
#include <iostream>
#include <memory>
#include <pthread.h>
#include <sched.h>
#include <string>
#include <thread>
#include <time.h>
#include <vector>

#include <controller/control.hpp>
#include <controller/dynamics.hpp>
#include <xarm_constants.hpp>
#include <xarm_port/xarm_init.hpp>
#include <yamlloader.hpp>

namespace {

enum class SessionMode {
    kUnilateral,
    kBilateral,
    kGravityComp,
};

struct TeleopSession {
    SessionMode mode = SessionMode::kUnilateral;
    std::atomic<bool> running{false};
    std::thread worker;
    std::thread leader_worker;
    std::thread follower_worker;
    std::thread admin_worker;

    xarm_sdk_handle_t leader_xarm = nullptr;
    xarm_sdk_handle_t follower_xarm = nullptr;

    std::unique_ptr<Dynamics> leader_dyn;
    std::unique_ptr<Dynamics> follower_dyn;
    std::shared_ptr<RobotSystemState> leader_state;
    std::shared_ptr<RobotSystemState> follower_state;
    std::unique_ptr<Control> leader_ctrl;
    std::unique_ptr<Control> follower_ctrl;

    std::string arm_side;

    std::vector<double> leader_kp;
    std::vector<double> leader_kd;
    std::vector<double> leader_fc;
    std::vector<double> leader_k;
    std::vector<double> leader_fv;
    std::vector<double> leader_fo;

    std::vector<double> follower_kp;
    std::vector<double> follower_kd;
    std::vector<double> follower_fc;
    std::vector<double> follower_k;
    std::vector<double> follower_fv;
    std::vector<double> follower_fo;

    // 关节状态回调（供外部如 ROS2 发布者使用）
    xarm_teleop_joint_state_cb_t joint_state_cb = nullptr;
    void *joint_state_cb_user_data = nullptr;

    // 双臂完整状态回调（同时提供 Leader + Follower arm/hand position）
    xarm_teleop_full_state_cb_t full_state_cb = nullptr;
    void *full_state_cb_user_data = nullptr;
};

thread_local std::string g_last_error;

inline void set_error(const std::string &msg) { g_last_error = msg; }

inline TeleopSession *cast_handle(xarm_teleop_handle_t h) {
    return reinterpret_cast<TeleopSession *>(h);
}

inline bool is_valid_arm_side(const std::string &arm_side) {
    return arm_side == "left_arm" || arm_side == "right_arm";
}

inline std::string config_file(const char *config_dir, const char *name) {
    return (std::filesystem::path(config_dir) / name).string();
}

inline void load_control_params(TeleopSession &sess, const char *config_dir) {
    YamlLoader leader_loader(config_file(config_dir, "leader.yaml"));
    YamlLoader follower_loader(config_file(config_dir, "follower.yaml"));

    sess.leader_kp = leader_loader.get_vector("LeaderArmParam", "Kp");
    sess.leader_kd = leader_loader.get_vector("LeaderArmParam", "Kd");
    sess.leader_fc = leader_loader.get_vector("LeaderArmParam", "Fc");
    sess.leader_k = leader_loader.get_vector("LeaderArmParam", "k");
    sess.leader_fv = leader_loader.get_vector("LeaderArmParam", "Fv");
    sess.leader_fo = leader_loader.get_vector("LeaderArmParam", "Fo");

    sess.follower_kp = follower_loader.get_vector("FollowerArmParam", "Kp");
    sess.follower_kd = follower_loader.get_vector("FollowerArmParam", "Kd");
    sess.follower_fc = follower_loader.get_vector("FollowerArmParam", "Fc");
    sess.follower_k = follower_loader.get_vector("FollowerArmParam", "k");
    sess.follower_fv = follower_loader.get_vector("FollowerArmParam", "Fv");
    sess.follower_fo = follower_loader.get_vector("FollowerArmParam", "Fo");
}

inline void sync_references(TeleopSession &sess) {
    auto leader_arm_resp = sess.leader_state->arm_state().get_all_responses();
    auto follower_arm_resp = sess.follower_state->arm_state().get_all_responses();

    auto leader_hand_resp = sess.leader_state->hand_state().get_all_responses();
    auto follower_hand_resp = sess.follower_state->hand_state().get_all_responses();

    // sess.leader_state->arm_state().set_all_references(follower_arm_resp);
    // sess.leader_state->hand_state().set_all_references(follower_hand_resp);

    sess.follower_state->arm_state().set_all_references(leader_arm_resp);
    sess.follower_state->hand_state().set_all_references(leader_hand_resp);
}

inline void shutdown_session_motors(TeleopSession &sess) {
    if (sess.leader_xarm != nullptr) {
        xarm_sdk_disable_all(sess.leader_xarm);
    }
    if (sess.follower_xarm != nullptr) {
        xarm_sdk_disable_all(sess.follower_xarm);
    }
}

inline void destroy_session_handles(TeleopSession &sess) {
    if (sess.leader_xarm != nullptr) {
        xarm_sdk_destroy(sess.leader_xarm);
        sess.leader_xarm = nullptr;
    }
    if (sess.follower_xarm != nullptr) {
        xarm_sdk_destroy(sess.follower_xarm);
        sess.follower_xarm = nullptr;
    }
}

inline void run_gravity_comp(TeleopSession *sess) {
    constexpr size_t ARM_DOF = 7;
    std::vector<double> arm_joint_positions(ARM_DOF, 0.0);
    std::vector<double> grav_torques(ARM_DOF, 0.0);

    while (sess->running) {
        std::vector<xarm_sdk_joint_state_t> sdk_states(ARM_DOF);
        xarm_sdk_get_arm_joint_states(sess->leader_xarm, sdk_states.data(), static_cast<int>(ARM_DOF));
        for (size_t i = 0; i < ARM_DOF; ++i) {
            arm_joint_positions[i] = static_cast<double>(sdk_states[i].pos);
        }

        sess->leader_dyn->GetGravity(arm_joint_positions.data(), grav_torques.data());

        std::vector<xarm_sdk_mit_param_t> cmds;
        cmds.reserve(grav_torques.size());
        for (double t : grav_torques) {
            xarm_sdk_mit_param_t p = {0.0f, 0.0f, 0.0f, 0.0f, static_cast<float>(t)};
            cmds.push_back(p);
        }

        xarm_sdk_arm_mit_control(sess->leader_xarm, cmds.data(), static_cast<int>(cmds.size()));
        xarm_sdk_recv_all(sess->leader_xarm, 1000);

        // 关节状态回调
        if (sess->joint_state_cb != nullptr) {
            float positions[ARM_DOF];
            for (size_t i = 0; i < ARM_DOF; ++i) {
                positions[i] = static_cast<float>(arm_joint_positions[i]);
            }
            sess->joint_state_cb(positions, static_cast<int>(ARM_DOF), 0.0f,
                                 sess->joint_state_cb_user_data);
        }
    }
}

// 设置当前线程为实时调度（SCHED_FIFO），失败时降级为普通调度并打印警告
inline void set_thread_realtime_priority(int priority = 50) {
    struct sched_param param;
    param.sched_priority = priority;
    int result = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
    if (result != 0) {
        std::cerr << "[WARN] Failed to set real-time priority (errno: " << result
                  << "). Run with sudo or setcap cap_sys_nice if needed." << std::endl;
    }
}

inline void run_leader_loop(TeleopSession *sess) {
    set_thread_realtime_priority(50);
    // 与原始 PeriodicTimerThread 一致：CLOCK_MONOTONIC + TIMER_ABSTIME 绝对定时，500 Hz
    constexpr long kPeriodNs = static_cast<long>(1e9 / FREQUENCY);  // 2,000,000 ns
    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);
    while (sess->running.load()) {
        if (sess->mode == SessionMode::kBilateral) {
            sess->leader_ctrl->bilateral_step();
        } else {
            sess->leader_ctrl->unilateral_step();
        }
        next_time.tv_nsec += kPeriodNs;
        while (next_time.tv_nsec >= 1000000000L) {
            next_time.tv_nsec -= 1000000000L;
            next_time.tv_sec += 1;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, nullptr);
    }
}

inline void run_follower_loop(TeleopSession *sess) {
    set_thread_realtime_priority(50);
    constexpr long kPeriodNs = static_cast<long>(1e9 / FREQUENCY);
    struct timespec next_time;
    clock_gettime(CLOCK_MONOTONIC, &next_time);
    while (sess->running.load()) {
        if (sess->mode == SessionMode::kBilateral) {
            sess->follower_ctrl->bilateral_step();
        } else {
            sess->follower_ctrl->unilateral_step();
        }
        next_time.tv_nsec += kPeriodNs;
        while (next_time.tv_nsec >= 1000000000L) {
            next_time.tv_nsec -= 1000000000L;
            next_time.tv_sec += 1;
        }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_time, nullptr);
    }
}

inline void run_admin_loop(TeleopSession *sess) {
    set_thread_realtime_priority(50);
    constexpr int ARM_DOF = 7;
    constexpr auto kPeriod = std::chrono::microseconds(2000);  // 500 Hz
    auto next_tick = std::chrono::steady_clock::now();

    while (sess->running.load()) {
        sync_references(*sess);

        // 关节状态回调
        if (sess->joint_state_cb != nullptr && sess->leader_state != nullptr) {
            auto leader_arm_resp = sess->leader_state->arm_state().get_all_responses();
            float positions[ARM_DOF] = {0.0f};
            const int n = std::min(static_cast<int>(leader_arm_resp.size()), ARM_DOF);
            for (int i = 0; i < n; ++i) {
                positions[i] = static_cast<float>(leader_arm_resp[i].position);
            }
            sess->joint_state_cb(positions, ARM_DOF, 0.0f, sess->joint_state_cb_user_data);
        }

        // 双臂完整状态回调（Leader + Follower arm/hand position）
        if (sess->full_state_cb != nullptr &&
            sess->leader_state != nullptr && sess->follower_state != nullptr) {
            auto leader_arm_resp    = sess->leader_state->arm_state().get_all_responses();
            auto leader_hand_resp   = sess->leader_state->hand_state().get_all_responses();
            auto follower_arm_resp  = sess->follower_state->arm_state().get_all_responses();
            auto follower_hand_resp = sess->follower_state->hand_state().get_all_responses();

            float leader_arm[ARM_DOF]    = {0.0f};
            float follower_arm[ARM_DOF]  = {0.0f};
            const int n = std::min(static_cast<int>(leader_arm_resp.size()), ARM_DOF);
            for (int i = 0; i < n; ++i) {
                leader_arm[i]   = static_cast<float>(leader_arm_resp[i].position);
                follower_arm[i] = static_cast<float>(follower_arm_resp.size() > static_cast<size_t>(i)
                    ? follower_arm_resp[i].position : 0.0);
            }
            const float leader_gripper   = leader_hand_resp.empty()   ? 0.0f
                : static_cast<float>(leader_hand_resp[0].position);
            const float follower_gripper = follower_hand_resp.empty() ? 0.0f
                : static_cast<float>(follower_hand_resp[0].position);

            sess->full_state_cb(leader_arm, ARM_DOF, leader_gripper,
                                follower_arm, follower_gripper,
                                sess->full_state_cb_user_data);
        }

        next_tick += kPeriod;
        std::this_thread::sleep_until(next_tick);
    }
}

inline void join_threads(TeleopSession *sess) {
    if (sess->worker.joinable()) {
        sess->worker.join();
    }
    if (sess->leader_worker.joinable()) {
        sess->leader_worker.join();
    }
    if (sess->follower_worker.joinable()) {
        sess->follower_worker.join();
    }
    if (sess->admin_worker.joinable()) {
        sess->admin_worker.join();
    }
}

inline void start_thread(TeleopSession *sess) {
    sess->running.store(true);
    sess->worker = std::thread([sess]() {
        try {
            run_gravity_comp(sess);
        } catch (const std::exception &e) {
            set_error(e.what());
            sess->running.store(false);
        }
    });
}

inline void start_dual_arm_threads(TeleopSession *sess) {
    sess->running.store(true);
    sess->leader_worker = std::thread([sess]() {
        try {
            run_leader_loop(sess);
        } catch (const std::exception &e) {
            set_error(e.what());
            sess->running.store(false);
        }
    });

    sess->follower_worker = std::thread([sess]() {
        try {
            run_follower_loop(sess);
        } catch (const std::exception &e) {
            set_error(e.what());
            sess->running.store(false);
        }
    });

    // 不需要延迟：AdjustPosition 末尾已通过 set_all_responses 把真实位置写入
    // responses，admin 线程第一次运行 sync_references 时可以读到正确值（INITIAL_POSITION）。
    // 延迟会导致 leader 在纯力矩模式下因重力补偿误差产生漂移，20ms 后 admin 把漂移后
    // 的 leader.response 赋给 follower.reference，造成 follower 往更大角度快速运动。
    sess->admin_worker = std::thread([sess]() {
        try {
            run_admin_loop(sess);
        } catch (const std::exception &e) {
            set_error(e.what());
            sess->running.store(false);
        }
    });
}

inline int create_dual_arm_session(SessionMode mode,
                                   const char *leader_can,
                                   const char *follower_can,
                                   const char *leader_urdf,
                                   const char *follower_urdf,
                                   const char *arm_side,
                                   const char *config_dir,
                                   xarm_teleop_handle_t *out) {
    if (leader_can == nullptr || follower_can == nullptr || leader_urdf == nullptr ||
        follower_urdf == nullptr || arm_side == nullptr || config_dir == nullptr || out == nullptr) {
        set_error("invalid null argument");
        return XARM_TELEOP_ERR_PARAM;
    }
    if (!std::filesystem::exists(leader_urdf) || !std::filesystem::exists(follower_urdf)) {
        set_error("urdf file not found");
        return XARM_TELEOP_ERR_FILE;
    }

    std::string arm_side_str = arm_side;
    if (!is_valid_arm_side(arm_side_str)) {
        set_error("arm_side must be left_arm or right_arm");
        return XARM_TELEOP_ERR_PARAM;
    }

    try {
        auto sess = std::make_unique<TeleopSession>();
        sess->mode = mode;
        sess->arm_side = arm_side_str;

        const std::string root_link = "xarm_body_link0";
        const std::string leaf_link = (arm_side_str == "left_arm") ? "xarm_left_hand" : "xarm_right_hand";

        sess->leader_dyn = std::make_unique<Dynamics>(leader_urdf, root_link, leaf_link);
        sess->follower_dyn = std::make_unique<Dynamics>(follower_urdf, root_link, leaf_link);
        sess->leader_dyn->Init();
        sess->follower_dyn->Init();

        sess->leader_xarm = xarm_init::XArmInitializer::initialize_xarm(leader_can, true);
        sess->follower_xarm = xarm_init::XArmInitializer::initialize_xarm(follower_can, true);

        sess->leader_state = std::make_shared<RobotSystemState>(7, 1);
        sess->follower_state = std::make_shared<RobotSystemState>(7, 1);

        sess->leader_ctrl = std::make_unique<Control>(
            sess->leader_xarm, sess->leader_dyn.get(), sess->follower_dyn.get(),
            sess->leader_state, 1.0 / FREQUENCY, ROLE_LEADER, arm_side_str, 7, 1);
        sess->follower_ctrl = std::make_unique<Control>(
            sess->follower_xarm, sess->leader_dyn.get(), sess->follower_dyn.get(),
            sess->follower_state, 1.0 / FREQUENCY, ROLE_FOLLOWER, arm_side_str, 7, 1);

        load_control_params(*sess, config_dir);
        sess->leader_ctrl->SetParameter(sess->leader_kp, sess->leader_kd,
                                        sess->leader_fc, sess->leader_k,
                                        sess->leader_fv, sess->leader_fo);
        sess->follower_ctrl->SetParameter(sess->follower_kp, sess->follower_kd,
                                          sess->follower_fc, sess->follower_k,
                                          sess->follower_fv, sess->follower_fo);

        std::thread leader_adjust(&Control::AdjustPosition, sess->leader_ctrl.get());
        std::thread follower_adjust(&Control::AdjustPosition, sess->follower_ctrl.get());
        leader_adjust.join();
        follower_adjust.join();

        *out = reinterpret_cast<xarm_teleop_handle_t>(sess.release());
        set_error("");
        return XARM_TELEOP_OK;
    } catch (const std::exception &e) {
        set_error(e.what());
        return XARM_TELEOP_ERR_INIT;
    }
}

}  // namespace

extern "C" {

int xarm_teleop_create_unilateral(const char *leader_can,
                                  const char *follower_can,
                                  const char *leader_urdf,
                                  const char *follower_urdf,
                                  const char *arm_side,
                                  const char *config_dir,
                                  xarm_teleop_handle_t *out) {
    return create_dual_arm_session(SessionMode::kUnilateral,
                                   leader_can,
                                   follower_can,
                                   leader_urdf,
                                   follower_urdf,
                                   arm_side,
                                   config_dir,
                                   out);
}

int xarm_teleop_create_bilateral(const char *leader_can,
                                 const char *follower_can,
                                 const char *leader_urdf,
                                 const char *follower_urdf,
                                 const char *arm_side,
                                 const char *config_dir,
                                 xarm_teleop_handle_t *out) {
    return create_dual_arm_session(SessionMode::kBilateral,
                                   leader_can,
                                   follower_can,
                                   leader_urdf,
                                   follower_urdf,
                                   arm_side,
                                   config_dir,
                                   out);
}

int xarm_teleop_create_gravity_comp(const char *can_if,
                                    const char *urdf_path,
                                    const char *config_dir,
                                    xarm_teleop_handle_t *out) {
    (void)config_dir;
    if (can_if == nullptr || urdf_path == nullptr || out == nullptr) {
        set_error("invalid null argument");
        return XARM_TELEOP_ERR_PARAM;
    }
    if (!std::filesystem::exists(urdf_path)) {
        set_error("urdf file not found");
        return XARM_TELEOP_ERR_FILE;
    }

    try {
        auto sess = std::make_unique<TeleopSession>();
        sess->mode = SessionMode::kGravityComp;

        sess->leader_dyn = std::make_unique<Dynamics>(
            urdf_path, "xarm_body_link0", "xarm_right_hand");
        sess->leader_dyn->Init();
        sess->leader_xarm = xarm_init::XArmInitializer::initialize_xarm(can_if, true);

        *out = reinterpret_cast<xarm_teleop_handle_t>(sess.release());
        set_error("");
        return XARM_TELEOP_OK;
    } catch (const std::exception &e) {
        set_error(e.what());
        return XARM_TELEOP_ERR_INIT;
    }
}

int xarm_teleop_start(xarm_teleop_handle_t h) {
    TeleopSession *sess = cast_handle(h);
    if (sess == nullptr) {
        set_error("invalid handle");
        return XARM_TELEOP_ERR_PARAM;
    }
    if (sess->running.load()) {
        set_error("session already running");
        return XARM_TELEOP_ERR_RUNNING;
    }
    try {
        if (sess->mode == SessionMode::kGravityComp) {
            start_thread(sess);
        } else {
            start_dual_arm_threads(sess);
        }
        set_error("");
        return XARM_TELEOP_OK;
    } catch (const std::exception &e) {
        set_error(e.what());
        return XARM_TELEOP_ERR_GENERAL;
    }
}

int xarm_teleop_stop(xarm_teleop_handle_t h) {
    TeleopSession *sess = cast_handle(h);
    if (sess == nullptr) {
        set_error("invalid handle");
        return XARM_TELEOP_ERR_PARAM;
    }
    sess->running.store(false);
    join_threads(sess);
    shutdown_session_motors(*sess);
    set_error("");
    return XARM_TELEOP_OK;
}

int xarm_teleop_is_running(xarm_teleop_handle_t h) {
    TeleopSession *sess = cast_handle(h);
    if (sess == nullptr) {
        set_error("invalid handle");
        return -1;
    }
    return sess->running.load() ? 1 : 0;
}

int xarm_teleop_wait(xarm_teleop_handle_t h) {
    TeleopSession *sess = cast_handle(h);
    if (sess == nullptr) {
        set_error("invalid handle");
        return XARM_TELEOP_ERR_PARAM;
    }
    join_threads(sess);
    set_error("");
    return XARM_TELEOP_OK;
}

int xarm_teleop_destroy(xarm_teleop_handle_t h) {
    TeleopSession *sess = cast_handle(h);
    if (sess == nullptr) {
        set_error("invalid handle");
        return XARM_TELEOP_ERR_PARAM;
    }

    sess->running.store(false);
    join_threads(sess);
    shutdown_session_motors(*sess);
    destroy_session_handles(*sess);

    delete sess;
    set_error("");
    return XARM_TELEOP_OK;
}

const char *xarm_teleop_get_last_error(void) { return g_last_error.c_str(); }

const char *xarm_teleop_version(void) { return "1.0.0"; }

int xarm_teleop_set_joint_state_callback(xarm_teleop_handle_t h,
                                          xarm_teleop_joint_state_cb_t cb,
                                          void *user_data) {
    TeleopSession *sess = cast_handle(h);
    if (sess == nullptr) {
        set_error("invalid handle");
        return XARM_TELEOP_ERR_PARAM;
    }
    sess->joint_state_cb = cb;
    sess->joint_state_cb_user_data = user_data;
    set_error("");
    return XARM_TELEOP_OK;
}

int xarm_teleop_set_full_state_callback(xarm_teleop_handle_t h,
                                         xarm_teleop_full_state_cb_t cb,
                                         void *user_data) {
    TeleopSession *sess = cast_handle(h);
    if (sess == nullptr) {
        set_error("invalid handle");
        return XARM_TELEOP_ERR_PARAM;
    }
    sess->full_state_cb = cb;
    sess->full_state_cb_user_data = user_data;
    set_error("");
    return XARM_TELEOP_OK;
}

int xarm_teleop_go_home(xarm_teleop_handle_t h) {
    TeleopSession *sess = cast_handle(h);
    if (sess == nullptr) {
        set_error("invalid handle");
        return XARM_TELEOP_ERR_PARAM;
    }
    if (sess->mode == SessionMode::kGravityComp) {
        set_error("go_home not supported in gravity compensation mode");
        return XARM_TELEOP_ERR_PARAM;
    }
    if (sess->leader_ctrl == nullptr || sess->follower_ctrl == nullptr) {
        set_error("control objects not initialized");
        return XARM_TELEOP_ERR_GENERAL;
    }
    try {
        const bool was_running = sess->running.load();
        if (was_running) {
            sess->running.store(false);
            join_threads(sess);
        }
        std::thread leader_adjust(&Control::AdjustPosition, sess->leader_ctrl.get());
        std::thread follower_adjust(&Control::AdjustPosition, sess->follower_ctrl.get());
        leader_adjust.join();
        follower_adjust.join();
        if (was_running) {
            start_dual_arm_threads(sess);
        }
        set_error("");
        return XARM_TELEOP_OK;
    } catch (const std::exception &e) {
        set_error(e.what());
        return XARM_TELEOP_ERR_GENERAL;
    }
}

}  // extern "C"
