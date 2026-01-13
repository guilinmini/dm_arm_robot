cmake_minimum_required(VERSION 3.8)
cmake_policy(SET CMP0072 NEW)
project(dm_mujoco)

# ===== C++ 标准 =====
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ===== 编译警告 =====
if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ===== ROS 2 依赖 =====
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# ==== MuJoCo 路径 =====
# set(MUJOCO_PATH $ENV{HOME}/.mujoco/mujoco210)
# set(MUJOCO_INCLUDE_DIR ${MUJOCO_PATH}/include)
# set(MUJOCO_LIB ${MUJOCO_PATH}/lib/libmujoco210.so)

include_directories(
  include
  /usr/local/include
  # ${mujoco_INCLUDE_DIRS}
)

add_executable(dm_mujoco_node
  src/dm_mujoco_node.cpp
)

ament_target_dependencies(dm_mujoco_node
  rclcpp
  std_msgs
)

target_link_libraries(dm_mujoco_node
  /usr/local/lib/libmujoco.so.3.4.1
  # ${mujoco_LIBRARIES}
)

install(TARGETS dm_mujoco_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
