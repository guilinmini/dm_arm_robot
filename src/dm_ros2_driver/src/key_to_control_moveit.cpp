#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <iostream>
#include <termios.h>
#include <unistd.h>

// 获取单个按键输入
char getch() {
    char buf = 0;
    struct termios old = {0};
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
        // 创建规划组的 MoveGroupInterface 对象
        arm_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "dm_arm");

        RCLCPP_INFO(this->get_logger(), "Planning frame: %s", arm_group_->getPlanningFrame().c_str());
        RCLCPP_INFO(this->get_logger(), "End effector link: %s", arm_group_->getEndEffectorLink().c_str());

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
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}