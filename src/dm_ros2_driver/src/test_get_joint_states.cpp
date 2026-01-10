#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "dm_ros2_driver/damiao.h"  // 假设的舵机控制库头文件（根据实际硬件替换）

using namespace damiao;

class DmArmController : public rclcpp::Node
{
public:
    DmArmController() : Node("get_joint_states_node")
    {
        auto serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);    // 初始化串口对象
        motor_control_ = std::make_unique<Motor_Control>(serial);               // 创建电机控制对象
        
        // 1.初始化七个电机对象
        for (int i = 0; i < 7; ++i) {
            // 这里的电机类型和ID需要根据实际情况修改
            motors_[i] = std::make_unique<Motor>(DM4310, static_cast<Motor_id>(i+1), 0x11+i);
            motor_control_->addMotor(motors_[i].get());
        }
        
        // 2.失能电机
        for (int i = 0; i < 7; ++i) {
            motor_control_->disable(*motors_[i]);
        }

        RCLCPP_INFO(this->get_logger(), "DM Arm Controller Initialized!");

        // 3.启动定时器，定时获取关节状态指令
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DmArmController::timer_callback, this));
    }
    
    // 获取电机的实际位置、速度和扭矩
    void timer_callback(){
        // 刷新电机状态
        for (int i = 0; i < 7; ++i) {
            motor_control_->refresh_motor_status(*motors_[i]);
            RCLCPP_INFO(this->get_logger(), "Motor ID: %d, Position: %.2f, Velocity: %.2f, Torque: %.2f",
                motors_[i]->GetSlaveId(), motors_[i]->Get_Position(), motors_[i]->Get_Velocity(), motors_[i]->Get_tau());
        }
    }

    // ************获取单个电机状态*************
    // ************需   ID-1
    void test_id(int ID){
        int i = ID;
        motors_[i-1] = std::make_unique<Motor>(DM4310, static_cast<Motor_id>(i), 0x10+i); // 主机ID不能相同
        motor_control_->addMotor(motors_[i-1].get());

        // 失能电机
        motor_control_->disable(*motors_[i-1]);

        // 获取电机的实际位置、速度和扭矩
        float current_position = motors_[i-1]->Get_Position();
        float current_velocity = motors_[i-1]->Get_Velocity();
        float current_torque = motors_[i-1]->Get_tau();

        motor_control_->refresh_motor_status(*motors_[i-1]);
        RCLCPP_INFO(this->get_logger(), "Motor ID: %d, Position: %.2f, Velocity: %.2f, Torque: %.2f",
                    motors_[i-1]->GetSlaveId(), current_position, current_velocity, current_torque);
    }


private:
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<Motor_Control> motor_control_;
    std::array<std::unique_ptr<Motor>, 7> motors_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DmArmController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}