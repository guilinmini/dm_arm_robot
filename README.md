# dm_arm_robot

**dm_arm_robot** 是针对达妙系列机械臂的 ROS 2 功能包集合。本项目提供了完整的 ROS 2 支持，包括 URDF 描述文件、MoveIt 2 运动规划配置、底层硬件驱动以及 C++ 控制示例。

## 功能特性 (Features)

*   **多自由度支持**：支持 6 自由度机械臂 + 抓夹（Gripper）控制。
*   **ROS 2 原生**：基于 ROS 2 humble 构建。
*   **MoveIt 2 集成**：提供完整的 MoveIt 2 配置，支持运动规划、轨迹控制和避障。
*   **硬件驱动**：包含 `dm_ros2_driver`，支持通过串口与达妙电机/控制器通讯。
*   **仿真与可视化**：支持 Rviz2 模型可视化。

## 目录结构 (Directory Structure)

*   `dm_arm_description`: 包含机械臂的 URDF 模型、Mesh 文件和可视化 Launch 文件。
*   `dm_arm_moveit_config`: MoveIt 2 的配置包，包含 SRDF、运动学参数和 MoveIt 启动文件。
*   `dm_ros2_driver`: 机械臂底层驱动节点，负责与硬件进行串口通信。
*   `moveit_control_cpp`: 使用 C++ API 控制 MoveIt 的示例代码（如画圆、位姿控制等）。

## 安装 (Installation)

1.  **创建工作空间** (如果已有工作空间可跳过):
    ```bash
    mkdir -p ~/dm_robot_ws
    cd ~/dm_robot_ws
    ```

2.  **克隆仓库**:
    ```bash
    git clone https://github.com/guilinmini/dm_arm_robot.git
    ```

3.  **编译**:
    ```bash
    colcon build --symlink-install
    source install/setup.bash
    ```

## 使用 (Usage)

### 1. 仅在 Rviz2 中查看模型
```bash
ros2 launch dm_arm_description display_rviz2.launch.py
```

### 2. 启动 MoveIt 2 (Demo 模式)
此模式不需要连接真实硬件，可用于模拟规划。
```bash
ros2 launch dm_arm_moveit_config demo.launch.py
```

### 3. 连接真实硬件 (需根据实际情况配置端口)
请确保您的用户有串口权限 (通常是 `/dev/ttyUSB*` 或 `/dev/ttyACM*`)。


## 更新日志 (Changelog)

### Recent Updates
1.  增加虚拟关节支持。
2.  修正关节限位 (Joint2, Joint3)。
3.  调整末端旋转角度。

