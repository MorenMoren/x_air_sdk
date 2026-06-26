#include <chrono>
#include <csignal>
#include <thread>
#include <xarm_teleop_sdk.h>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>

// 独立夹爪 CAN 控制（不依赖预编译 lib 内部结构）
extern int gripper_can_init(const char *can_if, int enable_fd);
extern int gripper_can_set_position(float pos, float kp, float kd);
extern void gripper_can_shutdown();

// ─── 双臂完整状态回调：将 Leader/Follower arm/hand position 桥接到 4 个 ROS2 话题 ──
struct FullStatePublisherCtx {
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr leader_arm_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr leader_hand_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr follower_arm_pub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr follower_hand_pub;
};

static void full_state_cb(const float *leader_arm, int arm_dof, float leader_gripper,
                           const float *follower_arm, float follower_gripper,
                           void *user_data) {
    auto *ctx = static_cast<FullStatePublisherCtx *>(user_data);
    if (ctx == nullptr) return;

    // Leader arm position
    if (ctx->leader_arm_pub) {
        std_msgs::msg::Float64MultiArray msg;
        msg.data.resize(arm_dof);
        for (int i = 0; i < arm_dof; ++i) msg.data[i] = static_cast<double>(leader_arm[i]);
        ctx->leader_arm_pub->publish(msg);
    }
    // Leader hand (gripper) position
    if (ctx->leader_hand_pub) {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {static_cast<double>(leader_gripper)};
        ctx->leader_hand_pub->publish(msg);
    }
    // Follower arm position
    if (ctx->follower_arm_pub) {
        std_msgs::msg::Float64MultiArray msg;
        msg.data.resize(arm_dof);
        for (int i = 0; i < arm_dof; ++i) msg.data[i] = static_cast<double>(follower_arm[i]);
        ctx->follower_arm_pub->publish(msg);
    }
    // Follower hand (gripper) position
    if (ctx->follower_hand_pub) {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {static_cast<double>(follower_gripper)};
        ctx->follower_hand_pub->publish(msg);
    }
}

// ─── ROS2 节点：归位服务 ────────────────────────────────────────────
class UnilateralControlNode : public rclcpp::Node {
public:
    UnilateralControlNode(xarm_teleop_handle_t handle, const std::string &arm_side)
        : Node("unilateral_control_node"), handle_(handle), arm_side_(arm_side) {
        home_service_ = this->create_service<std_srvs::srv::Trigger>(
            "robot_go_home",
            std::bind(&UnilateralControlNode::go_home_cb, this,
                      std::placeholders::_1, std::placeholders::_2));
        RCLCPP_INFO(this->get_logger(), "归位服务已就绪: /robot_go_home");
    }

private:
    void go_home_cb(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*req*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        RCLCPP_INFO(this->get_logger(), "执行归位...");
        int ret = xarm_teleop_go_home(handle_);
        if (ret == XARM_TELEOP_OK) {
            res->success = true;
            res->message = "已成功归位";
            RCLCPP_INFO(this->get_logger(), "归位完成");
        } else {
            res->success = false;
            res->message = xarm_teleop_get_last_error();
            RCLCPP_ERROR(this->get_logger(), "归位失败: %s", res->message.c_str());
        }
    }

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_service_;
    xarm_teleop_handle_t handle_;
    std::string arm_side_;
};

static xarm_teleop_handle_t g_handle = nullptr;
static volatile std::sig_atomic_t g_stop_requested = 0;
static void on_signal(int /*sig*/) { g_stop_requested = 1; }

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    std::signal(SIGTERM, on_signal);

    auto param_node = rclcpp::Node::make_shared("unilateral_control_ros2");
    param_node->declare_parameter<std::string>("leader_urdf_path", "");
    param_node->declare_parameter<std::string>("follower_urdf_path", "");
    param_node->declare_parameter<std::string>("arm_side", "right_arm");
    param_node->declare_parameter<std::string>("leader_can_if", "can0");
    param_node->declare_parameter<std::string>("follower_can_if", "can2");
    param_node->declare_parameter<std::string>("config_dir", "config");

    const std::string leader_urdf   = param_node->get_parameter("leader_urdf_path").as_string();
    const std::string follower_urdf = param_node->get_parameter("follower_urdf_path").as_string();
    const std::string arm_side      = param_node->get_parameter("arm_side").as_string();
    const std::string leader_can    = param_node->get_parameter("leader_can_if").as_string();
    const std::string follower_can  = param_node->get_parameter("follower_can_if").as_string();
    const std::string config_dir    = param_node->get_parameter("config_dir").as_string();

    if (leader_urdf.empty() || follower_urdf.empty()) {
        RCLCPP_ERROR(param_node->get_logger(), "leader_urdf_path / follower_urdf_path 不能为空");
        rclcpp::shutdown(); return 1;
    }
    if (arm_side != "left_arm" && arm_side != "right_arm") {
        RCLCPP_ERROR(param_node->get_logger(), "arm_side 无效: %s", arm_side.c_str());
        rclcpp::shutdown(); return 1;
    }

    RCLCPP_INFO(param_node->get_logger(), "=== XArm 单边遥操作 ROS2 (SDK %s) ===",
                xarm_teleop_version());
    RCLCPP_INFO(param_node->get_logger(), "arm_side: %s  leader_can: %s  follower_can: %s",
                arm_side.c_str(), leader_can.c_str(), follower_can.c_str());

    const std::string ns = (arm_side == "left_arm") ? "xarm_left" : "xarm_right";

    // 创建 4 个 Float64MultiArray 发布器
    // /xarm_{left|right}_leader/arm/position      — action（关节位置）
    // /xarm_{left|right}_leader/hand/position     — action.gripper（夹爪）
    // /xarm_{left|right}_follower/arm/position    — observation.state（关节位置）
    // /xarm_{left|right}_follower/hand/position   — observation.gripper_state（夹爪）
    FullStatePublisherCtx fs_ctx;
    fs_ctx.leader_arm_pub   = param_node->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/" + ns + "_leader/arm/position", 10);
    fs_ctx.leader_hand_pub  = param_node->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/" + ns + "_leader/hand/position", 10);
    fs_ctx.follower_arm_pub = param_node->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/" + ns + "_follower/arm/position", 10);
    fs_ctx.follower_hand_pub = param_node->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/" + ns + "_follower/hand/position", 10);

    int ret = xarm_teleop_create_unilateral(
        leader_can.c_str(), follower_can.c_str(),
        leader_urdf.c_str(), follower_urdf.c_str(),
        arm_side.c_str(), config_dir.c_str(), &g_handle);
    if (ret != XARM_TELEOP_OK) {
        RCLCPP_ERROR(param_node->get_logger(), "初始化失败: %s",
                     xarm_teleop_get_last_error());
        rclcpp::shutdown(); return 1;
    }

    xarm_teleop_set_full_state_callback(g_handle, full_state_cb, &fs_ctx);

    // ─── 夹爪 pad 控制初始化：独立 CAN 句柄 + 500Hz 线程 ───
    int gripper_ret = gripper_can_init(leader_can.c_str(), 1);
    if (gripper_ret != 0) {
        RCLCPP_WARN(param_node->get_logger(), "夹爪 CAN 初始化失败 (码=%d)", gripper_ret);
    } else {
        RCLCPP_INFO(param_node->get_logger(), "夹爪 CAN 已就绪 (%s, 50Hz)", leader_can.c_str());
    }

    // ─── 订阅外部夹爪指令（如手柄 pad），只更新目标值，实际发送在 full_state_cb 50Hz ───
    std::string gripper_topic = "/teleop_pad/" + ((arm_side == "left_arm") ? std::string("left") : std::string("right")) + "/gripper_command";
    auto gripper_sub = param_node->create_subscription<std_msgs::msg::Float64MultiArray>(
        gripper_topic, 10,
        [](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
            if (msg->data.empty())
                return;
            gripper_can_set_position(
                static_cast<float>(msg->data[0]), 0.0f, 0.0f);
        });
    RCLCPP_INFO(param_node->get_logger(),
                "夹爪指令订阅已就绪: %s", gripper_topic.c_str());

    auto control_node = std::make_shared<UnilateralControlNode>(g_handle, arm_side);

    ret = xarm_teleop_start(g_handle);
    if (ret != XARM_TELEOP_OK) {
        RCLCPP_ERROR(param_node->get_logger(), "启动失败: %s",
                     xarm_teleop_get_last_error());
        xarm_teleop_destroy(g_handle);
        rclcpp::shutdown(); return 1;
    }

    RCLCPP_INFO(param_node->get_logger(), "控制循环运行中，按 Ctrl+C 停止...");
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(param_node);
    executor.add_node(control_node);
    while (rclcpp::ok() && !g_stop_requested) {
        executor.spin_some();
        if (xarm_teleop_is_running(g_handle) != 1) {
            break;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    executor.cancel();

    if (xarm_teleop_is_running(g_handle) == 1) {
        xarm_teleop_stop(g_handle);
    }
    xarm_teleop_destroy(g_handle);
    g_handle = nullptr;
    rclcpp::shutdown();
    return 0;
}



