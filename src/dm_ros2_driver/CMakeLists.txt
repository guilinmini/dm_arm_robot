cmake_minimum_required(VERSION 3.8)
project(dm_ros2_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# add_executable(dm_ros2_control src/dm_ros2_control.cpp)
# target_include_directories(dm_ros2_control PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>)
# target_compile_features(dm_ros2_control PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
# ament_target_dependencies(
#   dm_ros2_control
#   "rclcpp"
#   "sensor_msgs"
# )

# add_executable(test_get_joint_states src/test_get_joint_states.cpp)
# ament_target_dependencies(
#   test_get_joint_states
#   "rclcpp"
#   "sensor_msgs"
# )

# add_executable(test_set_zero src/test_set_zero.cpp)
# ament_target_dependencies(
#   test_set_zero
#   "sensor_msgs"
#   "rclcpp"
# )

# add_executable(test_single_motor src/test_single_motor.cpp)
# ament_target_dependencies(
#   test_single_motor
#   "sensor_msgs"
#   "rclcpp"
# )


# 定义一个函数来添加可执行文件并设置包含目录
function(add_dm_executable target)
    add_executable(${target} src/${target}.cpp)
    target_include_directories(${target} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
    target_compile_features(${target} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
    ament_target_dependencies(
        ${target}
        "rclcpp"
        "sensor_msgs"
        "rclcpp_action"
        "control_msgs"
        "moveit_ros_planning_interface"
    )
endfunction()

# 添加可执行文件
add_dm_executable(dm_ros2_control)
add_dm_executable(test_get_joint_states)
add_dm_executable(test_set_zero)
add_dm_executable(test_single_motor)
add_dm_executable(dm_action_control)
add_dm_executable(key_to_control_dm)
add_dm_executable(moveit_pose)



install(TARGETS 
moveit_pose
  dm_ros2_control 
  test_single_motor 
  test_set_zero 
  test_get_joint_states 
  dm_action_control
  key_to_control_dm

  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
