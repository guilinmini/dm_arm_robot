#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class ArmController : public rclcpp::Node
{
public:
    ArmController() : Node("arm100_moveit_node"){
        // 初始化MoveGroupInterface
        // arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        //     shared_from_this(), "dm_arm");
        
        // RCLCPP_INFO(this->get_logger(), "Planning frame: %s", arm_->getPlanningFrame().c_str());
        // RCLCPP_INFO(this->get_logger(), "End effector link: %s", arm_->getEndEffectorLink().c_str());
    }

    void initialize(){
        arm_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "dm_arm");
        RCLCPP_INFO(this->get_logger(), "Planning frame: %s", arm_->getPlanningFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "End effector link: %s", arm_->getEndEffectorLink().c_str());
        
            // 获取规划组的基本信息
        RCLCPP_INFO(this->get_logger(),"初始化完成！");
    }

    void addObstacle()
    {
        // 创建碰撞物体
        moveit_msgs::msg::CollisionObject collision_object;
        collision_object.header.frame_id = arm_->getPlanningFrame();
        collision_object.id = "box1";

        // 定义物体形状
        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions.resize(3);
        primitive.dimensions[primitive.BOX_X] = 0.5;
        primitive.dimensions[primitive.BOX_Y] = 0.1;
        primitive.dimensions[primitive.BOX_Z] = 0.5;

        // 设置物体位姿
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = 0.2;
        box_pose.position.y = 0.2;
        box_pose.position.z = 0.3;

        collision_object.primitives.push_back(primitive);
        collision_object.primitive_poses.push_back(box_pose);
        collision_object.operation = collision_object.ADD;

        // 添加障碍物到场景
        planning_scene_interface_.applyCollisionObject(collision_object);
        RCLCPP_INFO(this->get_logger(), "Added collision object to the scene");
    }

    bool moveToTargetPose()
    {
        geometry_msgs::msg::Pose target_pose;
        // 转换为四元数
        tf2::Quaternion q;
        q.setRPY(M_PI, 0, 0);
        target_pose.orientation = tf2::toMsg(q); // 转换为geometry_msgs::msg::Quaternion
        // 设置目标位姿
        target_pose.position.x = -0.021;
        target_pose.position.y = -0.253;
        target_pose.position.z = 0.38;

        arm_->setPoseTarget(target_pose);

        // 规划并执行
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (arm_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Planning succeeded, executing trajectory");
            arm_->execute(plan);
            return true;
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
            return false;
        }
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto arm_controller = std::make_shared<ArmController>();

    // 添加障碍物
    arm_controller->initialize();
    arm_controller->addObstacle();
    rclcpp::sleep_for(std::chrono::seconds(1)); // 等待障碍物加载

    // 移动到目标位姿
    arm_controller->moveToTargetPose();

    rclcpp::shutdown();
    return 0;
}
