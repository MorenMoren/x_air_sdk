// Copyright 2025 vlai.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <chrono>
#include <string>
#include <vector>

#include <xarm_can_sdk.h>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "xarm_hardware/visibility_control.h"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace xarm_hardware {

/**
 * @brief Simplified XArm V10 Hardware Interface
 *
 * This is a simplified version that uses the XArm CAN API directly,
 * following the pattern from full_arm.cpp example. Much simpler than
 * the original implementation.
 */
class XArm_v10HW : public hardware_interface::SystemInterface {
 public:
  XArm_v10HW();
    ~XArm_v10HW() override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo& info) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces()
      override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces()
      override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;

  TEMPLATES__ROS2_CONTROL__VISIBILITY_PUBLIC
  hardware_interface::return_type write(
      const rclcpp::Time& time, const rclcpp::Duration& period) override;

 private:
  // V10 default configuration
  static constexpr size_t ARM_DOF = 7;
  static constexpr bool ENABLE_GRIPPER = true;

  // Default motor configuration for V10
  const std::vector<int> DEFAULT_MOTOR_TYPES = {
      XARM_SDK_MOTOR_DM8009,  // Joint 1
      XARM_SDK_MOTOR_DM8009,  // Joint 2
      XARM_SDK_MOTOR_DM4340,  // Joint 3
      XARM_SDK_MOTOR_DM4340,  // Joint 4
      XARM_SDK_MOTOR_DM4310,  // Joint 5
      XARM_SDK_MOTOR_DM4310,  // Joint 6
      XARM_SDK_MOTOR_DM4310   // Joint 7
  };

  const std::vector<uint32_t> DEFAULT_SEND_CAN_IDS = {0x01, 0x02, 0x03, 0x04,
                                                      0x05, 0x06, 0x07};
  const std::vector<uint32_t> DEFAULT_RECV_CAN_IDS = {0x11, 0x12, 0x13, 0x14,
                                                      0x15, 0x16, 0x17};

    const int DEFAULT_GRIPPER_MOTOR_TYPE = XARM_SDK_MOTOR_DM4310;
  const uint32_t DEFAULT_GRIPPER_SEND_CAN_ID = 0x08;
  const uint32_t DEFAULT_GRIPPER_RECV_CAN_ID = 0x18;

//   Default gains - Increased to compensate for lack of gravity compensation
//   const std::vector<double> DEFAULT_KP = {70.0, 50.0, 50.0, 50.0,
//                                           10.0, 10.0, 10.0, 0.5};
//   const std::vector<double> DEFAULT_KD = {0.8,  1.2,  1.2,  1.2,
//                                           0.3,  0.2,  0.3,  0.1};
// //   Default gains - Further increased for joints 1,4,7 with high gravity torque
  const std::vector<double> DEFAULT_KP = {240.0, 240.0, 240.0, 240.0, 24.0, 31.0, 25.0, 16.0};
  const std::vector<double> DEFAULT_KD = {3.0, 3.0, 3.0, 3.0, 0.2, 0.2, 0.2, 0.2};

//   const std::vector<double> DEFAULT_KP = {20.0, 20.0, 20.0, 20.0, 10.0, 5.0, 5.0, 5.0};
//   const std::vector<double> DEFAULT_KD = {0.5, 0.5, 0.5, 0.5, 0.2, 0.2, 0.2, 0.2};


  const double GRIPPER_JOINT_0_POSITION = 0.044;
  const double GRIPPER_JOINT_1_POSITION = 0.0;
  const double GRIPPER_MOTOR_0_RADIANS = 0.0;
  const double GRIPPER_MOTOR_1_RADIANS = -1.0472;
  const double GRIPPER_DEFAULT_KP = 5.0;
  const double GRIPPER_DEFAULT_KD = 0.1;

  // Joint position limits (rad) - from URDF/mechanical limits
  const std::vector<double> JOINT_LOWER_LIMITS = {
      -1.3,   // joint1
      -1.7,   // joint2
      -1.5,   // joint3
      0.0,    // joint4
      -1.5,   // joint5
      -0.7,   // joint6
      -1.5    // joint7
  };
  const std::vector<double> JOINT_UPPER_LIMITS = {
      3.4,    // joint1
      1.7,    // joint2
      1.5,    // joint3
      2.4,    // joint4
      1.5,    // joint5
      0.7,    // joint6
      1.5     // joint7
  };

  // Configuration
  std::string can_interface_;
  std::string arm_prefix_;
  bool hand_;
  bool can_fd_;

    // SDK session handle
    xarm_sdk_handle_t sdk_handle_;

  // Generated joint names for this arm instance
  std::vector<std::string> joint_names_;

  // ROS2 control state and command vectors
  std::vector<double> pos_commands_;
  std::vector<double> vel_commands_;
  std::vector<double> tau_commands_;
  std::vector<double> pos_states_;
  std::vector<double> vel_states_;
  std::vector<double> tau_states_;

  // Helper methods
  void return_to_zero();
  bool parse_config(const hardware_interface::HardwareInfo& info);
  void generate_joint_names();

  // Gripper mapping functions
  double joint_to_motor_radians(double joint_value);
  double motor_radians_to_joint(double motor_radians);
};

}  // namespace xarm_hardware
