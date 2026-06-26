/**
 * @file xarm_teleop_sdk.h
 * @brief XArm Teleop SDK 公开 C 接口
 *
 * 本头文件是 xarm_teleop 模块对外暴露的唯一公开接口。
 * 用户通过句柄（handle）方式创建、控制和销毁遥操作会话，
 * 无需关心内部 C++ 实现细节。
 *
 * 典型使用流程：
 * @code
 *   xarm_teleop_handle_t h = NULL;
 *   int ret = xarm_teleop_create_unilateral(
 *       "can0", "can2",
 *       "/tmp/leader.urdf", "/tmp/follower.urdf",
 *       "right_arm", "/path/to/config/",
 *       &h);
 *   if (ret != 0) { fprintf(stderr, "%s\n", xarm_teleop_get_last_error()); return 1; }
 *
 *   xarm_teleop_start(h);
 *   // ... 运行中，可等待 SIGINT 或调用 stop ...
 *   xarm_teleop_stop(h);
 *   xarm_teleop_destroy(h);
 * @endcode
 */

#ifndef XARM_TELEOP_SDK_H
#define XARM_TELEOP_SDK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>

/** 遥操作会话句柄，所有 API 的第一个参数 */
typedef void *xarm_teleop_handle_t;

/** API 返回值：成功 */
#define XARM_TELEOP_OK 0
/** API 返回值：通用失败 */
#define XARM_TELEOP_ERR_GENERAL -1
/** API 返回值：参数错误 */
#define XARM_TELEOP_ERR_PARAM -2
/** API 返回值：初始化失败（CAN / 电机初始化） */
#define XARM_TELEOP_ERR_INIT -3
/** API 返回值：文件不存在 */
#define XARM_TELEOP_ERR_FILE -4
/** API 返回值：会话已在运行 */
#define XARM_TELEOP_ERR_RUNNING -5

/* ──────────────────────────────────────────────────────────────────────────
 * 会话创建函数
 * ────────────────────────────────────────────────────────────────────────── */

/**
 * @brief 创建单边遥操作会话
 *
 * Leader 臂被动感知操作者意图；Follower 臂以位置模式跟随 Leader。
 * 调用本函数后会进行 CAN 初始化和电机使能，成功后返回就绪状态的句柄。
 *
 * @param leader_can    Leader 臂 CAN 接口名，如 "can0"
 * @param follower_can  Follower 臂 CAN 接口名，如 "can2"
 * @param leader_urdf   Leader 臂 URDF 文件路径（用于动力学建模）
 * @param follower_urdf Follower 臂 URDF 文件路径
 * @param arm_side      机械臂侧别："right_arm" 或 "left_arm"
 * @param config_dir    配置目录路径（包含 leader.yaml / follower.yaml）
 * @param[out] out      成功时写入句柄
 * @return XARM_TELEOP_OK (0) 表示成功，负值表示错误码
 */
int xarm_teleop_create_unilateral(const char *leader_can,
                                   const char *follower_can,
                                   const char *leader_urdf,
                                   const char *follower_urdf,
                                   const char *arm_side,
                                   const char *config_dir,
                                   xarm_teleop_handle_t *out);

/**
 * @brief 创建双边力反馈遥操作会话
 *
 * Leader 和 Follower 互相感知对方状态，实现透明力反馈遥操作。
 *
 * @param leader_can    Leader 臂 CAN 接口名
 * @param follower_can  Follower 臂 CAN 接口名
 * @param leader_urdf   Leader 臂 URDF 文件路径
 * @param follower_urdf Follower 臂 URDF 文件路径
 * @param arm_side      机械臂侧别："right_arm" 或 "left_arm"
 * @param config_dir    配置目录路径
 * @param[out] out      成功时写入句柄
 * @return XARM_TELEOP_OK (0) 表示成功，负值表示错误码
 */
int xarm_teleop_create_bilateral(const char *leader_can,
                                  const char *follower_can,
                                  const char *leader_urdf,
                                  const char *follower_urdf,
                                  const char *arm_side,
                                  const char *config_dir,
                                  xarm_teleop_handle_t *out);

/**
 * @brief 创建重力补偿会话（示教模式）
 *
 * 仅操作 Leader 臂，持续施加重力补偿力矩，使人员可以轻松拖动机械臂示教。
 *
 * @param can_if      CAN 接口名，如 "can0"
 * @param urdf_path   机械臂 URDF 文件路径
 * @param config_dir  配置目录路径
 * @param[out] out    成功时写入句柄
 * @return XARM_TELEOP_OK (0) 表示成功，负值表示错误码
 */
int xarm_teleop_create_gravity_comp(const char *can_if,
                                     const char *urdf_path,
                                     const char *config_dir,
                                     xarm_teleop_handle_t *out);

/* ──────────────────────────────────────────────────────────────────────────
 * 会话控制函数
 * ────────────────────────────────────────────────────────────────────────── */

/**
 * @brief 启动遥操作控制循环（非阻塞，内部创建控制线程）
 *
 * 在调用 start 之前，会话自动完成位置对齐（AdjustPosition），
 * 确保 Follower 臂移动到 Leader 臂当前位置后再开始控制。
 *
 * @param h 有效的遥操作会话句柄
 * @return XARM_TELEOP_OK (0) 表示成功
 */
int xarm_teleop_start(xarm_teleop_handle_t h);

/**
 * @brief 停止遥操作控制循环
 *
 * 停止后，电机进入失能状态（disable_all），会话句柄仍有效，
 * 可再次调用 start 重新启动（不重新初始化硬件）。
 *
 * @param h 有效的遥操作会话句柄
 * @return XARM_TELEOP_OK (0) 表示成功
 */
int xarm_teleop_stop(xarm_teleop_handle_t h);

/**
 * @brief 查询会话是否正在运行
 *
 * @param h 有效的遥操作会话句柄
 * @return 1 = 正在运行，0 = 已停止，负值 = 句柄无效
 */
int xarm_teleop_is_running(xarm_teleop_handle_t h);

/**
 * @brief 阻塞等待会话结束
 *
 * 挂起当前线程，直到会话被外部调用 stop 或因错误终止。
 * 等效于在 start 之后 join 内部线程。
 *
 * @param h 有效的遥操作会话句柄
 * @return XARM_TELEOP_OK (0) 表示成功
 */
int xarm_teleop_wait(xarm_teleop_handle_t h);

/**
 * @brief 销毁会话并释放所有资源
 *
 * 如果会话仍在运行，自动先调用 stop。
 * 调用后句柄无效，不可再使用。
 *
 * @param h 要销毁的会话句柄
 * @return XARM_TELEOP_OK (0) 表示成功
 */
int xarm_teleop_destroy(xarm_teleop_handle_t h);

/* ──────────────────────────────────────────────────────────────────────────
 * 错误查询
 * ────────────────────────────────────────────────────────────────────────── */

/**
 * @brief 获取最近一次 API 调用的错误信息字符串
 *
 * 返回的字符串由库内部管理，调用者不应释放。
 * 线程安全：每个线程有独立的错误缓冲区。
 *
 * @return 错误信息字符串，成功时返回空字符串 ""
 */
const char *xarm_teleop_get_last_error(void);

/**
 * @brief 获取 xarm_teleop SDK 版本字符串
 *
 * @return 形如 "1.0.0" 的版本字符串
 */
const char *xarm_teleop_version(void);

/* ──────────────────────────────────────────────────────────────────────────
 * 扩展 API：关节状态回调 & 归位
 * ────────────────────────────────────────────────────────────────────────── */

/**
 * @brief Leader 臂关节状态回调函数类型
 *
 * 由控制线程以约 500 Hz 的频率调用，可用于实时获取 Leader 臂状态。
 *
 * @param leader_positions  Leader 臂各关节位置数组（弧度），共 arm_dof 个元素
 * @param arm_dof           关节数量（通常为 7）
 * @param gripper_position  夹爪关节位置（弧度），重力补偿模式下为 0.0
 * @param user_data         注册时传入的用户数据指针
 */
typedef void (*xarm_teleop_joint_state_cb_t)(const float *leader_positions,
                                              int arm_dof,
                                              float gripper_position,
                                              void *user_data);

/**
 * @brief 注册 Leader 臂关节状态回调
 *
 * 设置后，每次控制循环迭代均调用该回调，提供 Leader 臂当前关节状态。
 * 适用于 ROS2 joint_states 发布等实时数据获取场景。
 * 传入 NULL 表示取消注册。
 *
 * 注意：回调在控制线程中同步调用，应尽量短小，避免阻塞控制循环。
 *
 * @param h         有效的遥操作会话句柄
 * @param cb        回调函数指针，可为 NULL（取消注册）
 * @param user_data 透传给回调的用户数据，可为 NULL
 * @return XARM_TELEOP_OK (0) 表示成功，负值表示错误码
 */
int xarm_teleop_set_joint_state_callback(xarm_teleop_handle_t h,
                                          xarm_teleop_joint_state_cb_t cb,
                                          void *user_data);

/**
 * @brief Leader + Follower 双臂完整状态回调函数类型
 *
 * 由 admin 控制线程以约 500 Hz 的频率调用，同时提供 Leader 和 Follower 两臂的
 * 关节位置与夹爪状态，用于 ROS2 话题发布等实时数据获取场景。
 *
 * @param leader_arm_positions    Leader 臂各关节位置数组（弧度），共 arm_dof 个元素
 * @param arm_dof                 关节数量（通常为 7）
 * @param leader_gripper          Leader 夹爪位置（弧度）
 * @param follower_arm_positions  Follower 臂各关节位置数组（弧度），共 arm_dof 个元素
 * @param follower_gripper        Follower 夹爪位置（弧度）
 * @param user_data               注册时传入的用户数据指针
 */
typedef void (*xarm_teleop_full_state_cb_t)(const float *leader_arm_positions,
                                             int arm_dof,
                                             float leader_gripper,
                                             const float *follower_arm_positions,
                                             float follower_gripper,
                                             void *user_data);

/**
 * @brief 注册 Leader + Follower 双臂完整状态回调
 *
 * 设置后，每次 admin 控制循环迭代均调用该回调，同时提供两臂状态。
 * 适用于需要同时发布 Leader/Follower arm/hand position 话题的场景。
 * 传入 NULL 表示取消注册。
 *
 * 注意：回调在控制线程中同步调用，应尽量短小，避免阻塞控制循环。
 * 仅对单边（unilateral）和双边（bilateral）模式有效；
 * 重力补偿模式（gravity_comp）下不调用该回调。
 *
 * @param h         有效的遥操作会话句柄
 * @param cb        回调函数指针，可为 NULL（取消注册）
 * @param user_data 透传给回调的用户数据，可为 NULL
 * @return XARM_TELEOP_OK (0) 表示成功，负值表示错误码
 */
int xarm_teleop_set_full_state_callback(xarm_teleop_handle_t h,
                                         xarm_teleop_full_state_cb_t cb,
                                         void *user_data);

/**
 * @brief 执行位置对齐（归位）
 *
 * 暂停当前控制循环，令 Follower 臂移动至 Leader 臂当前位置后自动恢复运行。
 * 适用于遥操作过程中需要重新同步两臂状态的场景。
 *
 * 仅对单边 / 双边模式会话有效；重力补偿模式返回 XARM_TELEOP_ERR_PARAM。
 * 若会话未在运行，则直接执行对齐动作后返回（不自动启动控制循环）。
 *
 * @param h 有效的遥操作会话句柄
 * @return XARM_TELEOP_OK (0) 表示成功，负值表示错误码
 */
int xarm_teleop_go_home(xarm_teleop_handle_t h);

#ifdef __cplusplus
}  // extern "C"
#endif

#endif  // XARM_TELEOP_SDK_H
