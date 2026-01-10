#include <memory>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h> 
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto const node = std::make_shared<rclcpp::Node>("dm_moveit_node",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

    auto const logger = rclcpp::get_logger("dm_moveit_node");   
    
    // 创建规划组的 MoveGroupInterface 对象
    using moveit::planning_interface::MoveGroupInterface;
    // auto gripper = MoveGroupInterface(node, "dm_hand");
    auto arm = MoveGroupInterface(node, "dm_arm");   

    // 获取规划组的基本信息
    RCLCPP_INFO(logger, "Planning frame: %s", arm.getPlanningFrame().c_str());      // 参考坐标系
    RCLCPP_INFO(logger, "End effector link: %s", arm.getEndEffectorLink().c_str()); // 末端执行器位置
    



    auto const target_pose = []{
        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.0;  
        msg.position.y = -0.3;
        msg.position.z = 0.3;
        msg.orientation.x = 0.7525521516799927;
        msg.orientation.y = -0.003340779570862651;
        msg.orientation.z = -0.020909862592816353;
        msg.orientation.w = 0.6581920981407166;
        return msg;
      }();
    
    arm.setPoseTarget(target_pose);
    
    // Create a plan to that target pose
    auto const [success, plan] = [&arm]{
        moveit::planning_interface::MoveGroupInterface::Plan msg;
        auto const ok = static_cast<bool>(arm.plan(msg));
        return std::make_pair(ok, msg);
    }();
    
    // Execute the plan
    if(success) {
        arm.execute(plan);
    } else {
        RCLCPP_ERROR(logger, "arm Planing failed!");
    }

    // // ========== 2. 控制夹爪 ==========
    // gripper.setNamedTarget("close"); // 设置预定义的目标位置 "hand_on"

    // // 执行运动
    // moveit::planning_interface::MoveGroupInterface::Plan hand_plan;
    // bool hand_success = (gripper.plan(hand_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    // if (hand_success)
    // {
    //     RCLCPP_INFO(logger, "hand Planning succeeded, executing...");
    //     gripper.execute(hand_plan);
    // }
    // else
    // {
    //     RCLCPP_ERROR(logger, "hand Planning failed!");
    // }



    rclcpp::shutdown();
    return 0;
}
