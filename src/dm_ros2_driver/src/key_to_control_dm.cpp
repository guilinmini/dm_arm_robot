#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

// 获取单个按键输入
char getch() {
    char buf = 0;
    struct termios old = {};
    if (tcgetattr(0, &old) < 0)
        perror("tcsetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 1;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0)
        perror("tcsetattr ICANON");
    if (read(0, &buf, 1) < 0)
        perror ("read()");
    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0)
        perror ("tcsetattr ~ICANON");
    return (buf);
}

class KeyControlArmNode : public rclcpp::Node
{
public:
    KeyControlArmNode() : Node("key_control_arm_node")
    {
    }
    void initialize() {
        // 现在节点已经构造完成，可以使用 shared_from_this()
        arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "dm_arm");

        RCLCPP_INFO(this->get_logger(), "Planning frame: %s", arm_group_->getPlanningFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "End effector link: %s", arm_group_->getEndEffectorLink().c_str());

        // 设置状态监控参数（关键修改）
        arm_group_->startStateMonitor(1.0);  // 设置更大的容忍时间（秒）
        arm_group_->setGoalJointTolerance(0.05);  // 增大关节容差
        
        // 强制刷新状态缓存
        arm_group_->getCurrentState()->update();
    
        printCurrentPose();
        // 启动按键监听线程
        key_listener_thread_ = std::thread(&KeyControlArmNode::listenForKeyInput, this);
    }


    ~KeyControlArmNode()
    {
        if (key_listener_thread_.joinable())
        {
            key_listener_thread_.join();
        }
    }

private:
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_group_;
    std::thread key_listener_thread_;

    void printCurrentPose()
    {
        try {
            auto current_pose = arm_group_->getCurrentPose().pose;
            RCLCPP_INFO(this->get_logger(), "Current End-Effector Pose:");
            RCLCPP_INFO(this->get_logger(), "Position: x=%.3f, y=%.3f, z=%.3f",
                        current_pose.position.x,
                        current_pose.position.y,
                        current_pose.position.z);
            RCLCPP_INFO(this->get_logger(), "Orientation: x=%.3f, y=%.3f, z=%.3f, w=%.3f",
                        current_pose.orientation.x,
                        current_pose.orientation.y,
                        current_pose.orientation.z,
                        current_pose.orientation.w);
            
            // 获取关节状态
            auto joint_values = arm_group_->getCurrentJointValues();
            RCLCPP_INFO(this->get_logger(), "Current Joint Values:");
            for (size_t i = 0; i < joint_values.size(); ++i) {
                RCLCPP_INFO(this->get_logger(), "Joint %zu: %.3f rad (%.1f deg)",
                           i, joint_values[i], joint_values[i] * (180.0/M_PI));
            }
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to get current pose: %s", e.what());
        }
    }

    void listenForKeyInput()
    {
        while (rclcpp::ok())
        {
            std::cout << "Press 'w' to move forward, 's' to move backward, 'q' to quit: ";
            char key = getch();

            if (key == 'w')
            {
                // 向前移动机械臂
                moveArmForward();
            }
            else if (key == 's')
            {
                // 向后移动机械臂
                moveArmBackward();
            }
            else if (key == 'q')
            {
                // 退出程序
                rclcpp::shutdown();
                break;
            }
        }
    }

    void moveArmForward()
    {
        geometry_msgs::msg::Pose current_pose = arm_group_->getCurrentPose().pose;
        current_pose.position.x += 0.1; // 向前移动 0.1 米

        arm_group_->setPoseTarget(current_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Planning succeeded, executing...");
            arm_group_->execute(plan);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        }
    }

    void moveArmBackward()
    {
        geometry_msgs::msg::Pose current_pose = arm_group_->getCurrentPose().pose;
        current_pose.position.x -= 0.1; // 向后移动 0.1 米

        arm_group_->setPoseTarget(current_pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        bool success = (arm_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

        if (success)
        {
            RCLCPP_INFO(this->get_logger(), "Planning succeeded, executing...");
            arm_group_->execute(plan);
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Planning failed!");
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KeyControlArmNode>();
     node->initialize();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}