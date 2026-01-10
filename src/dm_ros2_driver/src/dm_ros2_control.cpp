#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

// 假设的舵机控制库头文件（根据实际硬件替换）
#include "dm_ros2_driver/damiao.h"

using namespace damiao;

class DmArmController : public rclcpp::Node
{
public:
    DmArmController() : Node("hardware_interface_node")
    {
        // 初始化串口对象
        auto serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
        // 创建电机控制对象
        motor_control_ = std::make_unique<Motor_Control>(serial);
        
        // 初始化七个电机对象
        for (int i = 0; i < 7; ++i) {
            // 这里的电机类型和ID需要根据实际情况修改
            motors_[i] = std::make_unique<Motor>(DM4310, static_cast<Motor_id>(i+1), 0x11+i);
            motor_control_->addMotor(motors_[i].get());
        }
        
        // 失能电机并设置模式
        for (int i = 0; i < 7; ++i) {
            motor_control_->disable(*motors_[i]);
            motor_control_->switchControlMode(*motors_[i],POS_VEL_MODE);
        }
        
        // 打印电机初始位置
        RCLCPP_INFO(this->get_logger(), "***********************打印电机初始位置***********************");
        for (int i = 0; i < 7; ++i) {
            RCLCPP_INFO(this->get_logger(), "Motor ID: %d, Position: %.2f, Velocity: %.2f, Torque: %.2f",
                        motors_[i]->GetSlaveId(), motors_[i]->Get_Position(), motors_[i]->Get_Velocity(), motors_[i]->Get_tau());
        }

        // 使能所有电机
        for (int i = 0; i < 7; ++i) {
            motor_control_->enable(*motors_[i]);
        }

        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&DmArmController::jointstate_callback, this, std::placeholders::_1));

        // 初始化关节ID映射（根据实际配置填写）
        joint_id_map_ = {
            {"joint1", 1}, {"joint2", 2},
            {"joint3", 3}, {"joint4", 4},
            {"joint5", 5}, {"joint6", 6},
            {"joint7", 7},
        };
        RCLCPP_INFO(this->get_logger(), "DM Arm Controller Initialized!");
    }

    // 使能单个电机
    void enable_single_motor(int ID){
        motor_control_->enable(*motors_[ID-1]);
    }
    
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    std::unique_ptr<Motor_Control> motor_control_;
    std::array<std::unique_ptr<Motor>, 7> motors_;
    std::map<std::string, uint8_t> joint_id_map_; // 关节名到舵机ID的映射

    void jointstate_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // 遍历接收到的关节状态消息中的每个关节
        for (size_t i = 0; i < msg->name.size(); ++i) {
            const auto& joint_name = msg->name[i];
            // 检查关节名称是否在映射表中
            if (joint_id_map_.count(joint_name)) {
                uint8_t motor_id = joint_id_map_[joint_name];
                // 找到对应的电机对象
                auto& motor = motors_[motor_id - 1];

                // 获取关节位置信息
                float position = 0.0;
                float velocity = 0.1;
                if(motor_id == 2 || motor_id == 3){
                    position = -(msg->position[i]);
                }
                else{
                    position = msg->position[i];
                }
                
                // 获取速度信息
                // if (i < msg->velocity.size()) {
                //     if(motor_id == 2){
                //         velocity = -(msg->velocity[i]);
                //     }
                //     else{
                //         velocity = msg->velocity[i];
                //     }
                // }

                
                // 发送数据(单个关节)
                // if(motor_id == 1){
                //     RCLCPP_INFO(this->get_logger(), "To机械臂 Motor ID: %d, Position: %.2f, Velocity: %.2f",
                //         motor->GetSlaveId(), position, velocity);
                //     // motor_control_->control_pos_vel(*motor, position, velocity);
                // }
                    
                // 打印发送信息
                RCLCPP_INFO(this->get_logger(), "To机械臂 Motor ID: %d, Position: %.2f, Velocity: %.2f",
                    motor->GetSlaveId(), position, velocity);
                // 发送数据(七个关节)
                motor_control_->control_pos_vel(*motor, position, velocity);
                    
            } else {
                RCLCPP_WARN(this->get_logger(), "Unknown joint: %s", joint_name.c_str());
            }
        }
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DmArmController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}