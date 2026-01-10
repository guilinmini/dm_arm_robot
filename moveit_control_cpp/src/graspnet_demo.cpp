#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

class GraspnetArmController : public rclcpp::Node
{
public:
    GraspnetArmController():Node("graspnet_arm_controller_node"){
        subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/target_grasp_pose",10, 
            std::bind(&GraspnetArmController::graspnet_pose_callback, this, std::placeholders::_1));
        
        // arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
        //     shared_from_this(), "arm100_arm");

        RCLCPP_INFO(this->get_logger(),"graspnet_arm_controller_node Initialized!");
    }

    void initialize(){
        arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "arm100_arm");
        
            // 获取规划组的基本信息
        RCLCPP_INFO(this->get_logger(), "Planning frame: %s", arm_group_->getPlanningFrame().c_str());      // 参考坐标系
        RCLCPP_INFO(this->get_logger(), "End effector link: %s", arm_group_->getEndEffectorLink().c_str()); // 末端执行器位置
        RCLCPP_INFO(this->get_logger(),"Grsapnet Arm Controller Initialized! waiting for grasp poses ....");
    }
private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;

    void graspnet_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "Received grasp pose:");
        RCLCPP_INFO(this->get_logger(), "  Position: x=%.3f, y=%.3f, z=%.3f", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
        RCLCPP_INFO(this->get_logger(), "  Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f", msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z, msg->pose.orientation.w);
        
        arm_group_->setPoseTarget(*msg);
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool succese = (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
        if(succese){
            RCLCPP_INFO(this->get_logger(), "Planning succeeded, executing...");
            arm_group_->execute(plan);
            execute_grasp();
        }
    }

    void execute_grasp(){
        RCLCPP_INFO(this->get_logger(), "666666 completed");
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GraspnetArmController>();
    node->initialize();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}