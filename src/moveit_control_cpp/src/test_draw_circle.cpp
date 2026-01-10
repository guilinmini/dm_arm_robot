#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <geometry_msgs/msg/pose.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class CircleDrawer : public rclcpp::Node
{
public:
    CircleDrawer(const std::string &node_name = "circle_drawer",
                const std::string &group_name = "dm_arm",
                double default_radius = 0.1,
                int default_points = 30)
        : Node(node_name),
          move_group_(std::make_shared<moveit::planning_interface::MoveGroupInterface>(
              std::shared_ptr<rclcpp::Node>(this, [](auto*) {}), group_name)),
          radius_(default_radius),
          num_points_(default_points)
    {
        configure_move_group();
    }

    void set_circle_properties(double radius, int points)
    {
        radius_ = radius;
        num_points_ = points;
    }

    bool draw_horizontal_circle(const geometry_msgs::msg::Pose &center)
    {
        // 生成路径点
        auto waypoints = generate_horizontal_waypoints(center);

        // 笛卡尔路径规划
        moveit_msgs::msg::RobotTrajectory trajectory;
        double fraction = compute_cartesian_path(waypoints, trajectory);

        // 执行轨迹
        return execute_trajectory(trajectory, fraction);
    }

private:
    void configure_move_group()
    {
        move_group_->setMaxVelocityScalingFactor(0.3);
        move_group_->setPlannerId("RRTstar");
        move_group_->setPlanningTime(10.0);
    }

    std::vector<geometry_msgs::msg::Pose> generate_horizontal_waypoints(
        const geometry_msgs::msg::Pose &center)
    {
        std::vector<geometry_msgs::msg::Pose> waypoints;

        for (int i = 0; i <= num_points_; ++i)
        {
            double theta = 2 * M_PI * i / num_points_;
            geometry_msgs::msg::Pose pose = center;

            // 水平圆计算（XY平面）
            pose.position.x += radius_ * cos(theta);
            pose.position.y += radius_ * sin(theta);

            waypoints.push_back(pose);
        }
        return waypoints;
    }

    double compute_cartesian_path(const std::vector<geometry_msgs::msg::Pose> &waypoints,
                                  moveit_msgs::msg::RobotTrajectory &trajectory)
    {
        return move_group_->computeCartesianPath(
            waypoints,   // 路径点序列
            0.01,        // 步长（米）
            0.0,         // 跳跃阈值
            trajectory); // 输出轨迹
    }

    bool execute_trajectory(const moveit_msgs::msg::RobotTrajectory &trajectory,
                            double planning_fraction)
    {
        constexpr double SUCCESS_THRESHOLD = 0.9;

        if (planning_fraction >= SUCCESS_THRESHOLD)
        {
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            plan.trajectory_ = trajectory;
            return move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
        }

        RCLCPP_ERROR(this->get_logger(),
                     "轨迹规划失败，覆盖率: %.1f%%",
                     planning_fraction * 100);
        return false;
    }

    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    double radius_;
    int num_points_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    
    // CircleDrawer drawer;  // 直接实例化（非指针）
    // drawer.draw_horizontal_circle(center);  // 正确：对象用 .

    // 创建画圆控制器节点
    auto drawer = std::make_shared<CircleDrawer>();


    // 设置朝向
    tf2::Quaternion q;
    q.setRPY(0, 0, 0);  
    q.normalize();

    // 设置圆心参数
    geometry_msgs::msg::Pose center;
    center.position.x = 0.0;
    center.position.y = -0.4;
    center.position.z = 0.05; // 提升高度避免碰撞
    center.orientation = tf2::toMsg(q);

    // 可选：调整圆参数
    drawer->set_circle_properties(0.06, 50); // 半径15cm，40个路径点

    // 执行画圆操作
    bool success = drawer->draw_horizontal_circle(center);

    rclcpp::shutdown();
    return success ? 0 : 1;
}