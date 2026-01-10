#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class DmMoveitNode : public rclcpp::Node
{
public:
    DmMoveitNode() : Node("dm_moveit_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
    }

    void test()
    {
        // 初始化 MoveGroupInterface
        arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "dm_arm");

        // 打印规划组信息
        RCLCPP_INFO(get_logger(), "Planning frame: %s", arm_->getPlanningFrame().c_str());
        RCLCPP_INFO(get_logger(), "End effector link: %s", arm_->getEndEffectorLink().c_str());

        // 执行运动规划
        if (!move_to_target_pose())
        {
            RCLCPP_ERROR(get_logger(), "move_to_target_pose planning failed!");
        }
        if (!draw_circle())
        {
            RCLCPP_ERROR(get_logger(), "draw_circle planning failed!");
        }
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_;

    // 目标位置
    bool move_to_target_pose()
    {
        // 1. 设置目标位姿 设置朝向（绕X轴旋转-90度）

        geometry_msgs::msg::Pose msg;
        msg.position.x = 0.0;
        msg.position.y = -0.4;
        msg.position.z = 0.2;

        // 方案1：使用手动四元数（已验证可行的方向）
        // msg.orientation.x = -0.5052156448364258;
        // msg.orientation.y = 0.8628872036933899;
        // msg.orientation.z = -0.01302620954811573;
        // msg.orientation.w = -0.0036266790702939034;

        // 方案2：调整生成的旋转方向
        tf2::Quaternion q;
        q.setRPY(M_PI, 0, 0); // 绕 X 轴旋转 -180 度
        q.normalize();
        msg.orientation = tf2::toMsg(q);

        // 设置规划参数
        arm_->setPlanningTime(10.0);                // 规划时间
        arm_->setNumPlanningAttempts(5);            // 尝试次数
        arm_->setMaxVelocityScalingFactor(0.5);     // 速度限制
        arm_->setMaxAccelerationScalingFactor(0.3); // 加速度限制
        arm_->setGoalTolerance(0.01);               // 目标容差

        // 2. 设置目标并规划
        arm_->setPoseTarget(msg);
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        if (!arm_->plan(plan))
        {
            return false;
        }

        // 3. 执行规划
        return static_cast<bool>(arm_->execute(plan));
    }

    

    // 画圆功能
    bool draw_circle()
    {
        arm_->setPlanningTime(10.0); // 延长规划时间
        arm_->setMaxVelocityScalingFactor(0.3);
        arm_->setMaxAccelerationScalingFactor(0.3);

        // 定义圆心和半径
        const double radius = 0.05; // 确保半径在可达范围内
        const int num_points = 40;  // 增加路径点数量

        // 生成路径点（动态调整末端朝向）
        std::vector<geometry_msgs::msg::Pose> waypoints;
        for (int i = 0; i <= num_points; ++i)
        {
            double theta = 2 * M_PI * i / num_points;
            geometry_msgs::msg::Pose pose;
            pose.position.x = 0.0 + radius * cos(theta);
            pose.position.y = -0.4 + radius * sin(theta);
            pose.position.z = 0.2;

            tf2::Quaternion orientation;
            orientation.setRPY(M_PI, 0, theta);
            orientation.normalize();
            pose.orientation = tf2::toMsg(orientation);
            waypoints.push_back(pose);
        }

        // 检查路径点可达性
        for (const auto &pose : waypoints)
        {
            arm_->setPoseTarget(pose);
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            if (!arm_->plan(plan))
            {
                RCLCPP_ERROR(get_logger(), "Waypoint (%.2f, %.2f, %.2f) unreachable!",
                             pose.position.x, pose.position.y, pose.position.z);
                return false;
            }
        }

        // 笛卡尔路径规划（调整参数）
        moveit_msgs::msg::RobotTrajectory trajectory;
        double eef_step = 0.002;
        double jump_threshold = 0.5;
        double path_fraction = arm_->computeCartesianPath(
            waypoints, eef_step, jump_threshold, trajectory);

        if (path_fraction < 0.9)
        {
            RCLCPP_ERROR(get_logger(), "Cartesian path failed (%.1f%%)", path_fraction * 100);
            return false;
        }

        // 执行轨迹
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;
        return static_cast<bool>(arm_->execute(plan));
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DmMoveitNode>();
    node->test();
    rclcpp::shutdown();
    return 0;
}