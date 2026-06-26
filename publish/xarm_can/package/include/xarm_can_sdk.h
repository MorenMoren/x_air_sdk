#pragma once

#include <stdint.h>

#include "xarm_sdk_export.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void* xarm_sdk_handle_t;

enum {
    XARM_SDK_OK = 0,
    XARM_SDK_ERR_INVALID_ARGUMENT = -2,
    XARM_SDK_ERR_INVALID_STATE = -3,
    XARM_SDK_ERR_EXCEPTION = -4
};

enum {
    XARM_SDK_MOTOR_DM4310 = 0,
    XARM_SDK_MOTOR_DM4340 = 1,
    XARM_SDK_MOTOR_DM6006 = 2,
    XARM_SDK_MOTOR_DM8006 = 3,
    XARM_SDK_MOTOR_DM8009 = 4,
    XARM_SDK_MOTOR_DM10010L = 5,
    XARM_SDK_MOTOR_DM10010 = 6,
    XARM_SDK_MOTOR_DM1015 = 7,
    XARM_SDK_MOTOR_DMH3510 = 8,
    XARM_SDK_MOTOR_DM_J4310_2EC = 9
};

typedef struct xarm_sdk_mit_param_s {
    float pos;
    float vel;
    float kp;
    float kd;
    float torque;
} xarm_sdk_mit_param_t;

typedef struct xarm_sdk_joint_state_s {
    float pos;
    float vel;
    float torque;
} xarm_sdk_joint_state_t;

XARM_SDK_API const char* xarm_sdk_get_version(void);
XARM_SDK_API int xarm_sdk_create(const char* can_if, int enable_fd, xarm_sdk_handle_t* out);
XARM_SDK_API int xarm_sdk_destroy(xarm_sdk_handle_t h);

XARM_SDK_API int xarm_sdk_init_arm_motors(xarm_sdk_handle_t h, const int* motor_types,
                                          const uint32_t* send_ids,
                                          const uint32_t* recv_ids, int count);

XARM_SDK_API int xarm_sdk_init_gripper_motor(xarm_sdk_handle_t h, int motor_type,
                                             uint32_t send_id, uint32_t recv_id);

XARM_SDK_API int xarm_sdk_enable_all(xarm_sdk_handle_t h);
XARM_SDK_API int xarm_sdk_disable_all(xarm_sdk_handle_t h);
XARM_SDK_API int xarm_sdk_set_zero_all(xarm_sdk_handle_t h);
XARM_SDK_API int xarm_sdk_refresh_all(xarm_sdk_handle_t h);
XARM_SDK_API int xarm_sdk_recv_all(xarm_sdk_handle_t h, int timeout_us);
XARM_SDK_API int xarm_sdk_set_callback_mode_state_all(xarm_sdk_handle_t h);
XARM_SDK_API int xarm_sdk_get_arm_joint_states(xarm_sdk_handle_t h,
                                                xarm_sdk_joint_state_t* states,
                                                int count);
XARM_SDK_API int xarm_sdk_get_gripper_state(xarm_sdk_handle_t h,
                                            xarm_sdk_joint_state_t* state);

XARM_SDK_API int xarm_sdk_gripper_open(xarm_sdk_handle_t h, float kp, float kd);
XARM_SDK_API int xarm_sdk_gripper_close(xarm_sdk_handle_t h, float kp, float kd);
XARM_SDK_API int xarm_sdk_gripper_mit_control(xarm_sdk_handle_t h,
                                              const xarm_sdk_mit_param_t* param);

XARM_SDK_API int xarm_sdk_arm_mit_control(xarm_sdk_handle_t h, const xarm_sdk_mit_param_t* params,
                                          int count);

XARM_SDK_API int xarm_sdk_get_last_error(char* buffer, int buffer_len);

#ifdef __cplusplus
}
#endif
