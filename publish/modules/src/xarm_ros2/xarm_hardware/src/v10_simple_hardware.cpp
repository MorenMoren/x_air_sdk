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

#include "xarm_hardware/v10_simple_hardware.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"

namespace xarm_hardware {

namespace {

bool sdk_call_ok(int ret, const char* api_name) {
  if (ret == XARM_SDK_OK) {
    return true;
  }
  char err_buf[512] = {0};
  xarm_sdk_get_last_error(err_buf, static_cast<int>(sizeof(err_buf)));
  RCLCPP_ERROR(rclcpp::get_logger("XArm_v10HW"), "%s failed ret=%d, last_error=%s", api_name,
               ret, err_buf);
  return false;
}

}  // namespace

XArm_v10HW::XArm_v10HW() : sdk_handle_(nullptr) {}

XArm_v10HW::~XArm_v10HW() {
  if (sdk_handle_) {
    xarm_sdk_destroy(sdk_handle_);
    sdk_handle_ = nullptr;
  }
}

bool XArm_v10HW::parse_config(const hardware_interface::HardwareInfo& info) {
  // Parse CAN interface (default: can0)
  auto it = info.hardware_parameters.find("can_interface");
  can_interface_ = (it != info.hardware_parameters.end()) ? it->second : "can0";

  // Parse arm prefix (default: empty for single arm, "left_" or "right_" for
  // bimanual)
  it = info.hardware_parameters.find("arm_prefix");
  arm_prefix_ = (it != info.hardware_parameters.end()) ? it->second : "";

  // Parse gripper enable (default: true for V10)
  it = info.hardware_parameters.find("hand");
  if (it == info.hardware_parameters.end()) {
    hand_ = true;  // Default to true for V10
  } else {
    // Handle both "true"/"True" and "false"/"False"
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    hand_ = (value == "true");
  }

  // Parse CAN-FD enable (default: true for V10)
  it = info.hardware_parameters.find("can_fd");
  if (it == info.hardware_parameters.end()) {
    can_fd_ = true;  // Default to true for V10
  } else {
    // Handle both "true"/"True" and "false"/"False"
    std::string value = it->second;
    std::transform(value.begin(), value.end(), value.begin(), ::tolower);
    can_fd_ = (value == "true");
  }

  RCLCPP_INFO(rclcpp::get_logger("XArm_v10HW"),
              "Configuration: CAN=%s, arm_prefix=%s, hand=%s, can_fd=%s",
              can_interface_.c_str(), arm_prefix_.c_str(),
              hand_ ? "enabled" : "disabled", can_fd_ ? "enabled" : "disabled");
  return true;
}

void XArm_v10HW::generate_joint_names() {
  joint_names_.clear();
  // TODO: read from urdf properly and sort in the future.
  // Currently, the joint names are hardcoded for order consistency to align
  // with hardware. Generate arm joint names: xarm_{arm_prefix}joint{N}
  for (size_t i = 1; i <= ARM_DOF; ++i) {
    std::string joint_name =
        "xarm_" + arm_prefix_ + "joint" + std::to_string(i);
    joint_names_.push_back(joint_name);
  }

  // Generate gripper joint name if enabled
  if (hand_) {
    std::string gripper_joint_name = "xarm_" + arm_prefix_ + "finger_joint1";
    joint_names_.push_back(gripper_joint_name);
    RCLCPP_INFO(rclcpp::get_logger("XArm_v10HW"), "Added gripper joint: %s",
                gripper_joint_name.c_str());
  } else {
    RCLCPP_INFO(rclcpp::get_logger("XArm_v10HW"),
                "Gripper joint NOT added because hand_=false");
  }

  RCLCPP_INFO(rclcpp::get_logger("XArm_v10HW"),
              "Generated %zu joint names for arm prefix '%s'",
              joint_names_.size(), arm_prefix_.c_str());
}

hardware_interface::CallbackReturn XArm_v10HW::on_init(
    const hardware_interface::HardwareInfo& info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  // Parse configuration
  if (!parse_config(info)) {
    return CallbackReturn::ERROR;
  }

  // Generate joint names based on arm prefix
  generate_joint_names();

  // Validate joint count (7 arm joints + optional gripper)
  size_t expected_joints = ARM_DOF + (hand_ ? 1 : 0);
  if (joint_names_.size() != expected_joints) {
    RCLCPP_ERROR(rclcpp::get_logger("XArm_v10HW"),
                 "Generated %zu joint names, expected %zu", joint_names_.size(),
                 expected_joints);
    return CallbackReturn::ERROR;
  }

  // Initialize XArm with configurable CAN-FD setting
  RCLCPP_INFO(rclcpp::get_logger("XArm_v10HW"),
              "Initializing XArm on %s with CAN-FD %s...",
              can_interface_.c_str(), can_fd_ ? "enabled" : "disabled");
  if (sdk_handle_) {
    xarm_sdk_destroy(sdk_handle_);
    sdk_handle_ = nullptr;
  }
  if (!sdk_call_ok(
          xarm_sdk_create(can_interface_.c_str(), can_fd_ ? 1 : 0, &sdk_handle_),
          "xarm_sdk_create")) {
    return CallbackReturn::ERROR;
  }

  // Initialize arm motors with V10 defaults
  if (!sdk_call_ok(xarm_sdk_init_arm_motors(
                       sdk_handle_, DEFAULT_MOTOR_TYPES.data(),
                       DEFAULT_SEND_CAN_IDS.data(), DEFAULT_RECV_CAN_IDS.data(),
                       static_cast<int>(DEFAULT_MOTOR_TYPES.size())),
                   "xarm_sdk_init_arm_motors")) {
    return CallbackReturn::ERROR;
  }

  // Initialize gripper if enabled
  if (hand_) {
    RCLCPP_INFO(rclcpp::get_logger("XArm_v10HW"), "Initializing gripper...");
    if (!sdk_call_ok(
            xarm_sdk_init_gripper_motor(sdk_handle_, DEFAULT_GRIPPER_MOTOR_TYPE,
                                        DEFAULT_GRIPPER_SEND_CAN_ID,
                                        DEFAULT_GRIPPER_RECV_CAN_ID),
            "xarm_sdk_init_gripper_motor")) {
      return CallbackReturn::ERROR;
    }
  }

  // Initialize state and command vectors based on generated joint count
  const size_t total_joints = joint_names_.size();
  pos_commands_.resize(total_joints, 0.0);
  vel_commands_.resize(total_joints, 0.0);
  tau_commands_.resize(total_joints, 0.0);
  pos_states_.resize(total_joints, 0.0);
  vel_states_.resize(total_joints, 0.0);
  tau_states_.resize(total_joints, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("XArm_v10HW"),
              "XArm V10 Simple HW initialized successfully");

  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn XArm_v10HW::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  // Set callback mode to ignore during configuration
  if (!sdk_call_ok(xarm_sdk_refresh_all(sdk_handle_), "xarm_sdk_refresh_all")) {
    return CallbackReturn::ERROR;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  if (!sdk_call_ok(xarm_sdk_recv_all(sdk_handle_, 0), "xarm_sdk_recv_all")) {
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
XArm_v10HW::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &pos_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY, &vel_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
XArm_v10HW::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // TODO: consider exposing only needed interfaces to avoid undefined behavior.
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_POSITION,
        &pos_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_VELOCITY,
        &vel_commands_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint_names_[i], hardware_interface::HW_IF_EFFORT, &tau_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn XArm_v10HW::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("XArm_v10HW"), "Activating XArm V10...");
  if (!sdk_call_ok(xarm_sdk_set_callback_mode_state_all(sdk_handle_),
                   "xarm_sdk_set_callback_mode_state_all")) {
    return CallbackReturn::ERROR;
  }
  if (!sdk_call_ok(xarm_sdk_enable_all(sdk_handle_), "xarm_sdk_enable_all")) {
    return CallbackReturn::ERROR;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  if (!sdk_call_ok(xarm_sdk_recv_all(sdk_handle_, 0), "xarm_sdk_recv_all")) {
    return CallbackReturn::ERROR;
  }

  // Return to zero position
  return_to_zero();

  RCLCPP_INFO(rclcpp::get_logger("XArm_v10HW"), "XArm V10 activated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn XArm_v10HW::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("XArm_v10HW"),
              "Deactivating XArm V10...");

  // Disable all motors (like full_arm.cpp exit)
  if (!sdk_call_ok(xarm_sdk_disable_all(sdk_handle_), "xarm_sdk_disable_all")) {
    return CallbackReturn::ERROR;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  if (!sdk_call_ok(xarm_sdk_recv_all(sdk_handle_, 0), "xarm_sdk_recv_all")) {
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("XArm_v10HW"), "XArm V10 deactivated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type XArm_v10HW::read(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Receive all motor states
  if (!sdk_call_ok(xarm_sdk_refresh_all(sdk_handle_), "xarm_sdk_refresh_all") ||
      !sdk_call_ok(xarm_sdk_recv_all(sdk_handle_, 0), "xarm_sdk_recv_all")) {
    return hardware_interface::return_type::ERROR;
  }

  // Read arm joint states
  std::vector<xarm_sdk_joint_state_t> arm_states(ARM_DOF);
  if (!sdk_call_ok(xarm_sdk_get_arm_joint_states(
                       sdk_handle_, arm_states.data(),
                       static_cast<int>(arm_states.size())),
                   "xarm_sdk_get_arm_joint_states")) {
    return hardware_interface::return_type::ERROR;
  }
  for (size_t i = 0; i < ARM_DOF; ++i) {
    pos_states_[i] = arm_states[i].pos;
    vel_states_[i] = arm_states[i].vel;
    tau_states_[i] = arm_states[i].torque;
  }

  // Read gripper state if enabled
  if (hand_ && joint_names_.size() > ARM_DOF) {
    xarm_sdk_joint_state_t gripper_state{};
    if (sdk_call_ok(xarm_sdk_get_gripper_state(sdk_handle_, &gripper_state),
                    "xarm_sdk_get_gripper_state")) {
      pos_states_[ARM_DOF] = motor_radians_to_joint(gripper_state.pos);
      vel_states_[ARM_DOF] = gripper_state.vel;
      tau_states_[ARM_DOF] = gripper_state.torque;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type XArm_v10HW::write(
    const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  // Control arm motors with MIT control
  std::vector<xarm_sdk_mit_param_t> arm_params;
  for (size_t i = 0; i < ARM_DOF; ++i) {
    // 🔒 安全限幅：确保位置在安全范围内
    double safe_position = pos_commands_[i];
    if (safe_position < JOINT_LOWER_LIMITS[i]) {
      // RCLCPP_WARN_THROTTLE(
      //     rclcpp::get_logger("XArm_v10HW"),
      //     *rclcpp::Clock().get_clock(),
      //     5000,  // 5秒打印一次
      //     "Joint %zu command %.3f below lower limit %.3f, clamping",
      //     i, safe_position, JOINT_LOWER_LIMITS[i]);
      safe_position = JOINT_LOWER_LIMITS[i];
    }
    if (safe_position > JOINT_UPPER_LIMITS[i]) {
      // RCLCPP_WARN_THROTTLE(
      //     rclcpp::get_logger("XArm_v10HW"),
      //     *rclcpp::Clock().get_clock(),
      //     5000,  // 5秒打印一次
      //     "Joint %zu command %.3f above upper limit %.3f, clamping",
      //     i, safe_position, JOINT_UPPER_LIMITS[i]);
      safe_position = JOINT_UPPER_LIMITS[i];
    }
    
    arm_params.push_back({static_cast<float>(safe_position),
                          static_cast<float>(vel_commands_[i]),
                          static_cast<float>(DEFAULT_KP[i]),
                          static_cast<float>(DEFAULT_KD[i]),
                          static_cast<float>(tau_commands_[i])});
  }
  if (!sdk_call_ok(xarm_sdk_arm_mit_control(sdk_handle_, arm_params.data(),
                                            static_cast<int>(arm_params.size())),
                   "xarm_sdk_arm_mit_control")) {
    return hardware_interface::return_type::ERROR;
  }
  // Control gripper if enabled
  if (hand_ && joint_names_.size() > ARM_DOF) {
    // TODO the true mappings are unimplemented.
    double motor_command = joint_to_motor_radians(pos_commands_[ARM_DOF]);
    xarm_sdk_mit_param_t gripper_param{static_cast<float>(motor_command), 0.0F,
                                       static_cast<float>(GRIPPER_DEFAULT_KP),
                                       static_cast<float>(GRIPPER_DEFAULT_KD),
                                       0.0F};
    if (!sdk_call_ok(xarm_sdk_gripper_mit_control(sdk_handle_, &gripper_param),
                     "xarm_sdk_gripper_mit_control")) {
      return hardware_interface::return_type::ERROR;
    }
  }
  if (!sdk_call_ok(xarm_sdk_recv_all(sdk_handle_, 1000), "xarm_sdk_recv_all")) {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

// void XArm_v10HW::return_to_zero() {
//   RCLCPP_INFO(rclcpp::get_logger("XArm_v10HW"),
//               "Returning to zero position...");

//   // Return arm to zero with MIT control
//   std::vector<xarm::damiao_motor::MITParam> arm_params;
//   for (size_t i = 0; i < ARM_DOF; ++i) {
//     arm_params.push_back({DEFAULT_KP[i], DEFAULT_KD[i], 0.0, 0.0, 0.0});
//   }
//   xarm_->get_arm().mit_control_all(arm_params);

//   // Return gripper to zero if enabled
//   if (hand_) {
//     xarm_->get_gripper().mit_control_all(
//         {{GRIPPER_DEFAULT_KP, GRIPPER_DEFAULT_KD, GRIPPER_JOINT_0_POSITION,
//         0.0,
//           0.0}});
//   }
//   std::this_thread::sleep_for(std::chrono::microseconds(1000));
//   xarm_->recv_all();
// }
void XArm_v10HW::return_to_zero() {
  RCLCPP_INFO(rclcpp::get_logger("XArm_v10HW"),
              "Returning to initial position...");

  // Initial position values (from your deployment script)
  // const std::vector<double> INITIAL_POSITIONS = {
  //     -0.1413366903181501,   // joint1
  //     0.14400701915007197,   // joint2
  //     -0.2534905012588702,   // joint3
  //     0.8703364614328226,    // joint4
  //     0.012397955291065799,  // joint5
  //     0.12722209506370596,   // joint6
  //     0.9061951628900591     // joint7
  // };
  const std::vector<double> INITIAL_POSITIONS = {
      0,   // joint1
      0,   // joint2
      0,   // joint3
      0,    // joint4
      0,  // joint5
      0,   // joint6
      0     // joint7
  };

  // Return arm to initial position with MIT control
  std::vector<xarm_sdk_mit_param_t> arm_params;
  for (size_t i = 0; i < ARM_DOF; ++i) {
    arm_params.push_back({static_cast<float>(INITIAL_POSITIONS[i]), 0.0F,
                          static_cast<float>(DEFAULT_KP[i]),
                          static_cast<float>(DEFAULT_KD[i]), 0.0F});
  }
  if (!sdk_call_ok(xarm_sdk_arm_mit_control(sdk_handle_, arm_params.data(),
                                            static_cast<int>(arm_params.size())),
                   "xarm_sdk_arm_mit_control")) {
    return;
  }

  // Return gripper to zero if enabled
  if (hand_) {
    xarm_sdk_mit_param_t gripper_param{0.0F, 0.0F,
                                       static_cast<float>(GRIPPER_DEFAULT_KP),
                                       static_cast<float>(GRIPPER_DEFAULT_KD),
                                       0.0F};
    sdk_call_ok(xarm_sdk_gripper_mit_control(sdk_handle_, &gripper_param),
                "xarm_sdk_gripper_mit_control");
  }
  std::this_thread::sleep_for(std::chrono::microseconds(1000));
  sdk_call_ok(xarm_sdk_recv_all(sdk_handle_, 0), "xarm_sdk_recv_all");
}

// Gripper mapping helper functions
double XArm_v10HW::joint_to_motor_radians(double joint_value) {
  // Joint 0=closed -> motor 0 rad, Joint 0.044=open -> motor -1.0472 rad
  return (joint_value / GRIPPER_JOINT_0_POSITION) *
         GRIPPER_MOTOR_1_RADIANS;  // Scale from 0-0.044 to 0 to -1.0472
}

double XArm_v10HW::motor_radians_to_joint(double motor_radians) {
  // Motor 0 rad=closed -> joint 0, Motor -1.0472 rad=open -> joint 0.044
  return GRIPPER_JOINT_0_POSITION *
         (motor_radians /
          GRIPPER_MOTOR_1_RADIANS);  // Scale from 0 to -1.0472 to 0-0.044
}

}  // namespace xarm_hardware

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(xarm_hardware::XArm_v10HW,
                       hardware_interface::SystemInterface)
