cmake_minimum_required(VERSION 3.5)
project(segway_msgs)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpic")
set(CMAKE_C_FLAGS   "${CMAKE_C_FLAGS}   -fpic")
# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
####find_package(builtin_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

set(msg_files
  "msg/bms_fb.msg"
  "msg/chassis_ctrl_src_fb.msg"
  "msg/chassis_mileage_meter_fb.msg"
  "msg/chassis_mode_fb.msg"
  "msg/error_code_fb.msg"
  "msg/front_axle_angle_fb.msg"
  "msg/motor_work_mode_fb.msg"
  "msg/speed_fb.msg"
  "msg/ticks_fb.msg"
)

set(srv_files
  "srv/ros_stop_chassis_rotate_cmd.srv"
  "srv/ros_start_chassis_right_rotate_cmd.srv"
  "srv/ros_start_chassis_left_rotate_cmd.srv"
  "srv/ros_set_vel_max_cmd.srv"
  "srv/ros_set_load_param_cmd.srv"
  "srv/ros_set_chassis_poweroff_cmd.srv"
  "srv/ros_set_chassis_buzzer_cmd.srv"
  "srv/ros_set_cfg_rotate_function_cmd.srv"
  "srv/ros_reset_host_power_cmd.srv"
  "srv/ros_get_vel_max_feedback_cmd.srv"
  "srv/ros_get_sw_version_cmd.srv"
  "srv/ros_get_rotate_function_cfg_cmd.srv"
  "srv/ros_get_load_param_cmd.srv"
  "srv/ros_get_host_and_chassis_match_cmd.srv"
  "srv/ros_get_chassis_SN_cmd.srv"
  "srv/ros_get_chassis_rotate_switch_cmd.srv"
  "srv/ros_enable_chassis_rotate_cmd.srv"
  "srv/ros_clear_chassis_error_code_cmd.srv"
)

set(action_files
  "action/ros_set_iap_cmd.action"
)
rosidl_generate_interfaces(${PROJECT_NAME} 
  ${msg_files} 
  ${srv_files}
  ${action_files}
  DEPENDENCIES  geometry_msgs std_msgs ADD_LINTER_TESTS)

# ament_export_dependencies(rosidl_default_generators)
# ament_export_dependencies(rosidl_default_runtime)
# ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
