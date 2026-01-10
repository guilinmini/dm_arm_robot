#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>

double degrees_to_radians(double degrees)
{
    return degrees * M_PI / 180.0;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>("moveit_joint_gripper_pose",
                                                     rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    auto const logger = rclcpp::get_logger("moveit_joint_gripper_pose");

    // 创建规划组的 MoveGroupInterface 对象
    using moveit::planning_interface::MoveGroupInterface;
    auto gripper = MoveGroupInterface(node, "arm100_hand");
    auto arm = MoveGroupInterface(node, "arm100_arm");

    gripper.setGoalJointTolerance(0.01); // 设置关节运动的允许误差，单位弧度

    arm.setGoalJointTolerance(0.01);          // 设置关节运动的允许误差，单位弧度
    arm.setMaxAccelerationScalingFactor(0.8); // 限制速度
    arm.setMaxVelocityScalingFactor(0.8);     // 限制加速度

    try
    {
        // ========== 1. 控制机械臂关节 ==========
        std::vector<double> target_joint_positions{
            degrees_to_radians(0.1), // joint1
            degrees_to_radians(50.0), // joint2
            degrees_to_radians(0.1), // joint3
            degrees_to_radians(0.0), // joint4
            degrees_to_radians(0.0), // joint5
        };
        arm.setJointValueTarget(target_joint_positions);

        // 规划并执行机械臂运动
        moveit::planning_interface::MoveGroupInterface::Plan arm_plan;
        bool arm_success = (arm.plan(arm_plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (arm_success)
        {
            RCLCPP_INFO(logger, "Arm planning succeeded, executing...");
            arm.execute(arm_plan);
        }
        else
        {
            RCLCPP_ERROR(logger, "Arm planning failed!");
            rclcpp::shutdown();
            return 1;
        }

        // ========== 2. 控制夹爪 ==========
        gripper.setNamedTarget("open"); // 设置预定义的目标位置 "hand_on"

        // 执行运动
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (gripper.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(logger, "Planning succeeded, executing...");
            gripper.execute(plan);
        }
        else
        {
            RCLCPP_ERROR(logger, "Planning failed!");
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(logger, "Exception caught: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
