/**
 * @file gripper_bridge.cpp
 * @brief 独立 CAN 句柄 + 500Hz 线程，纯发不收，不干扰遥操
 */
#include "xarm_can_sdk.h"
#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include <iostream>

static xarm_sdk_handle_t g_handle = nullptr;
static std::mutex g_mutex;
static std::atomic<float> g_target{-0.5f};
static std::atomic<bool> g_running{false};
static std::thread g_thread;
static std::atomic<int> g_count{-1};

int gripper_can_init(const char *can_if, int enable_fd) {
    std::lock_guard<std::mutex> lock(g_mutex);
    if (g_handle != nullptr)
        return 0;

    int ret = xarm_sdk_create(can_if, enable_fd, &g_handle);
    if (ret != XARM_SDK_OK)
        return ret;

    ret = xarm_sdk_init_gripper_motor(g_handle, XARM_SDK_MOTOR_DM4310, 0x08, 0x18);
    if (ret != XARM_SDK_OK) {
        xarm_sdk_destroy(g_handle);
        g_handle = nullptr;
        return ret;
    }

    xarm_sdk_set_callback_mode_state_all(g_handle);
    xarm_sdk_enable_all(g_handle);

    // 启动 500Hz 发送线程
    g_running.store(true);
    g_thread = std::thread([]() {
        constexpr auto kInterval = std::chrono::microseconds(2000);
        while (g_running.load()) {
            int count = g_count.load();
            if (count >= 0 && count <= 1500) {
                float pos = g_target.load();
                {
                    std::lock_guard<std::mutex> lock(g_mutex);
                    if (g_handle) {
                        xarm_sdk_mit_param_t p = {pos, 0.0f, 16.0f, 0.2f, 0.0f};
                        //if(pos!=g_target.load()) std::cout<<"count==0:"<<pos<<"time:"<<std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count()<<std::endl;
                        xarm_sdk_gripper_mit_control(g_handle, &p);
                        
                        
                    }
                }
                g_count.store(count + 1);
            }
            std::this_thread::sleep_for(kInterval);
        }
    });
    return XARM_SDK_OK;
}

int gripper_can_set_position(float pos, float, float) {
    std::lock_guard<std::mutex> lock(g_mutex);
    //if(pos!=g_target.load())
    	//std::cout<<"gripper can set:"<<pos<<"time:"<<std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count()<<std::endl;
    g_target.store(pos);
    g_count.store(0);
    return 0;
}

void gripper_can_shutdown() {
    g_running.store(false);
    if (g_thread.joinable())
        g_thread.join();
    std::lock_guard<std::mutex> lock(g_mutex);
    if (g_handle) {
        xarm_sdk_disable_all(g_handle);
        xarm_sdk_destroy(g_handle);
        g_handle = nullptr;
    }
}
