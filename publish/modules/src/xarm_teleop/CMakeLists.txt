# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.22)
project(xarm_teleop)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------
# Find external packages
# -----------------------------
# Find xarm_can SDK (C ABI - closed source friendly)
if(DEFINED ENV{XARM_SDK_ROOT})
  set(XARM_SDK_ROOT $ENV{XARM_SDK_ROOT})
elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../xarm_can/sdk/package")
  set(XARM_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../xarm_can/sdk/package")
elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../../xarm_can/package")
  set(XARM_SDK_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../xarm_can/package")
else()
  message(FATAL_ERROR "Cannot find xarm_can SDK. Set XARM_SDK_ROOT or build from source.")
endif()
message(STATUS "Using xarm_can SDK from: ${XARM_SDK_ROOT}")

# Resolve arch-specific lib dir: prefer lib/<arch>/ if it exists, fall back to lib/
if(EXISTS "${XARM_SDK_ROOT}/lib/${CMAKE_SYSTEM_PROCESSOR}/libxarm_can_sdk.so")
  set(XARM_SDK_LIB_DIR "${XARM_SDK_ROOT}/lib/${CMAKE_SYSTEM_PROCESSOR}")
else()
  set(XARM_SDK_LIB_DIR "${XARM_SDK_ROOT}/lib")
endif()
message(STATUS "Using xarm_can SDK lib dir: ${XARM_SDK_LIB_DIR}")

add_library(xarm_can_sdk SHARED IMPORTED)
set_target_properties(xarm_can_sdk PROPERTIES
  IMPORTED_LOCATION "${XARM_SDK_LIB_DIR}/libxarm_can_sdk.so"
  INTERFACE_INCLUDE_DIRECTORIES "${XARM_SDK_ROOT}/include"
)
# find_package(urdf REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(urdfdom REQUIRED)
find_package(urdfdom_headers REQUIRED)
find_package(yaml-cpp REQUIRED)
if(NOT TARGET yaml-cpp::yaml-cpp AND TARGET yaml-cpp)
  add_library(yaml-cpp::yaml-cpp ALIAS yaml-cpp)
endif()


# -----------------------------
# Find ROS2 dependencies
# -----------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED) 
find_package(geometry_msgs REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
# -----------------------------
# 自适应构建 xarm_teleop_lib
# 模式1 源码模式：src/controller/control.cpp 存在 → 从全部源码编译
# 模式2 发布模式：只有 sdk_impl.cpp + prebuilt/xarm_teleop/lib/libxarm_teleop_lib.so
#               → sdk_impl.cpp 的所有符号已在预编译 .so 中，可直接使用预编译版本
# 模式3 纯预编译模式：../../prebuilt/xarm_teleop/lib/libxarm_teleop_lib.so
# -----------------------------
set(_INTERNAL_CTRL "${CMAKE_CURRENT_SOURCE_DIR}/src/controller/control.cpp")
# Prefer arch-specific prebuilt, fall back to flat layout
set(_PREBUILT_LIB_LOCAL_ARCH "${CMAKE_CURRENT_SOURCE_DIR}/prebuilt/xarm_teleop/lib/${CMAKE_SYSTEM_PROCESSOR}/libxarm_teleop_lib.so")
set(_PREBUILT_LIB_LOCAL      "${CMAKE_CURRENT_SOURCE_DIR}/prebuilt/xarm_teleop/lib/libxarm_teleop_lib.so")
set(_PREBUILT_LIB_ARCH "${CMAKE_CURRENT_SOURCE_DIR}/../../prebuilt/xarm_teleop/lib/${CMAKE_SYSTEM_PROCESSOR}/libxarm_teleop_lib.so")
set(_PREBUILT_LIB  "${CMAKE_CURRENT_SOURCE_DIR}/../../prebuilt/xarm_teleop/lib/libxarm_teleop_lib.so")
if(EXISTS "${_PREBUILT_LIB_LOCAL_ARCH}")
  set(_PREBUILT_LIB_LOCAL "${_PREBUILT_LIB_LOCAL_ARCH}")
endif()
if(EXISTS "${_PREBUILT_LIB_ARCH}")
  set(_PREBUILT_LIB "${_PREBUILT_LIB_ARCH}")
endif()

if(EXISTS "${_INTERNAL_CTRL}")
  # ── 模式1：源码模式 ──────────────────────────────────────────────
  message(STATUS "[xarm_teleop] 从源码编译 libxarm_teleop_lib")
  add_library(
    xarm_teleop_lib SHARED
    src/controller/dynamics.cpp
    src/controller/control.cpp
    src/controller/control_ros2_publisher.cpp
    src/xarm_port/xarm_init.cpp
    src/xarm_port/joint_mapper.cpp
    src/xarm_teleop_sdk_impl.cpp
  )
  target_include_directories(xarm_teleop_lib
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
      $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )
  target_link_libraries(
    xarm_teleop_lib
    xarm_can_sdk
    Eigen3::Eigen
    ${orocos_kdl_LIBRARIES}
    kdl_parser::kdl_parser
    urdfdom::urdfdom_model
    yaml-cpp::yaml-cpp
  )
  ament_target_dependencies(
    xarm_teleop_lib
    rclcpp
    sensor_msgs
    std_msgs
    geometry_msgs
  )
elseif(EXISTS "${_PREBUILT_LIB_LOCAL}")
  # ── 模式2：发布模式（publish 包内 prebuilt）──────────────────────
  # xarm_teleop_sdk_impl.cpp 已编入预编译 .so，直接使用预编译版本
  message(STATUS "[xarm_teleop] 使用发布预编译库: ${_PREBUILT_LIB_LOCAL}")
  add_library(xarm_teleop_lib SHARED IMPORTED GLOBAL)
  set_target_properties(xarm_teleop_lib PROPERTIES
    IMPORTED_LOCATION "${_PREBUILT_LIB_LOCAL}"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include"
    INTERFACE_LINK_LIBRARIES "xarm_can_sdk"
  )
elseif(EXISTS "${_PREBUILT_LIB}")
  # ── 模式3：纯预编译模式（外部 prebuilt）─────────────────────────
  message(STATUS "[xarm_teleop] 使用外部预编译库: ${_PREBUILT_LIB}")
  add_library(xarm_teleop_lib SHARED IMPORTED GLOBAL)
  set_target_properties(xarm_teleop_lib PROPERTIES
    IMPORTED_LOCATION "${_PREBUILT_LIB}"
    INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/include"
    INTERFACE_LINK_LIBRARIES "xarm_can_sdk"
  )
else()
  message(FATAL_ERROR "[xarm_teleop] 找不到内部源码也找不到预编译库，无法构建。")
endif()
# -----------------------------
# Executables (SDK-only, no internal headers)
# -----------------------------
set(_EXECUTABLE_TARGETS)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/control/gravity_compasation.cpp")
  add_executable(gravity_comp control/gravity_compasation.cpp)
  target_link_libraries(gravity_comp PRIVATE xarm_teleop_lib)
  list(APPEND _EXECUTABLE_TARGETS gravity_comp)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/control/xarm_unilateral_control.cpp")
  add_executable(unilateral_control control/xarm_unilateral_control.cpp)
  target_link_libraries(unilateral_control PRIVATE xarm_teleop_lib)
  list(APPEND _EXECUTABLE_TARGETS unilateral_control)
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/control/xarm_bilateral_control.cpp")
  add_executable(bilateral_control control/xarm_bilateral_control.cpp)
  target_link_libraries(bilateral_control PRIVATE xarm_teleop_lib)
  list(APPEND _EXECUTABLE_TARGETS bilateral_control)
endif()

# comm_test only available in source build
if(EXISTS "${_INTERNAL_CTRL}")
  add_executable(comm_test control/xarm_communication_test.cpp)
  target_link_libraries(comm_test PRIVATE xarm_teleop_lib)
endif()

# -----------------------------
# Executables（ROS2 enabled executables）
# -----------------------------

# -----------------------------
# ROS2-enabled executable
# -----------------------------
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/control/xarm_uilateral_control_ros2.cpp")
  add_executable(unilateral_control_ros2 
    control/xarm_uilateral_control_ros2.cpp
    control/gripper_bridge.cpp
  )
  
  # 先添加 ROS2 依赖
  ament_target_dependencies(unilateral_control_ros2
    rclcpp
    sensor_msgs
    std_msgs
    std_srvs
    geometry_msgs
  )

  # 再链接自定义库
  target_link_libraries(unilateral_control_ros2 xarm_teleop_lib)
  list(APPEND _EXECUTABLE_TARGETS unilateral_control_ros2)
endif()

# -----------------------------
# Install targets
# -----------------------------

# Install library（仅源码模式下安装，IMPORTED 目标不可 install）
if(EXISTS "${_INTERNAL_CTRL}")
  install(
    TARGETS xarm_teleop_lib
    EXPORT xarm_teleop_targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
elseif(EXISTS "${_PREBUILT_LIB_LOCAL}")
  # 模式2：将包内预编译 .so 拷贝到安装目录
  install(
    FILES "${_PREBUILT_LIB_LOCAL}"
    DESTINATION lib
  )
elseif(EXISTS "${_PREBUILT_LIB}")
  # 模式3：将外部预编译 .so 拷贝到安装目录
  install(
    FILES "${_PREBUILT_LIB}"
    DESTINATION lib
  )
endif()

# Install executables
if(_EXECUTABLE_TARGETS)
  install(
    TARGETS ${_EXECUTABLE_TARGETS}
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Install public SDK headers only
install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.h"
)

# Install launch files
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING
  PATTERN "*.py"
  PATTERN "*.launch"
  PATTERN "*.xml"
)

# Install config files
install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
  FILES_MATCHING
  PATTERN "*.yaml"
  PATTERN "*.yml"
)

# Install script files
install(
  DIRECTORY script/
  DESTINATION share/${PROJECT_NAME}/script
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING
  PATTERN "*.sh"
  PATTERN "*.bash"
)

# Export targets（仅源码模式，预编译模式无 CMake target 可导出）
if(EXISTS "${_INTERNAL_CTRL}")
  install(
    EXPORT xarm_teleop_targets
    FILE xarm_teleopTargets.cmake
    NAMESPACE xarm_teleop::
    DESTINATION share/${PROJECT_NAME}/cmake
  )
endif()

# -----------------------------
# ament package configuration
# -----------------------------
if(EXISTS "${_INTERNAL_CTRL}")
  ament_export_targets(xarm_teleop_targets HAS_LIBRARY_TARGET)
endif()
ament_export_dependencies(
  rclcpp
  sensor_msgs
  std_srvs  
  std_msgs
  geometry_msgs
  Eigen3
  orocos_kdl
  kdl_parser
  urdfdom
  yaml-cpp
)

ament_package()