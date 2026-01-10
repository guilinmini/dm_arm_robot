#include <rclcpp/rclcpp.hpp>
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
        
        // 1.初始化七个电机对象
        for (int i = 0; i < 7; ++i) {
            // 这里的电机类型和ID需要根据实际情况修改
            motors_[i] = std::make_unique<Motor>(DM4310, static_cast<Motor_id>(i+1), 0x11+i); // 主机ID不能相同
            motor_control_->addMotor(motors_[i].get());
        }
        
        // 2.失能电机
        for (int i = 0; i < 7; ++i) {
            motor_control_->disable(*motors_[i]);
        }
        
        // 打印电机初始位置
        RCLCPP_INFO(this->get_logger(), "***********************打印电机初始位置***********************");
        for (int i = 0; i < 7; ++i) {
            RCLCPP_INFO(this->get_logger(), "Motor ID: %d, Position: %.2f, Velocity: %.2f, Torque: %.2f",
                        motors_[i]->GetSlaveId(), motors_[i]->Get_Position(), motors_[i]->Get_Velocity(), motors_[i]->Get_tau());
        }
        
        // 3.设置电机零点
        for (int i = 0; i < 7; ++i) {
            motor_control_->set_zero_position(*motors_[i]);
        }

        // 4.保存参数
        for (int i = 0; i < 7; ++i) {
            motor_control_->save_motor_param(*motors_[i]);
        }

        RCLCPP_INFO(this->get_logger(), "成功设置当前位置为零点！并保存！");
        
        // 打印零点位置
        RCLCPP_INFO(this->get_logger(), "***********************打印电机零点位置***********************");
        for (int i = 0; i < 7; ++i) {
            motor_control_->refresh_motor_status(*motors_[i]);
            RCLCPP_INFO(this->get_logger(), "Motor ID: %d, Position: %.2f, Velocity: %.2f, Torque: %.2f",
                        motors_[i]->GetSlaveId(), motors_[i]->Get_Position(), motors_[i]->Get_Velocity(), motors_[i]->Get_tau());
        }
    }
private:
    std::unique_ptr<Motor_Control> motor_control_;
    std::array<std::unique_ptr<Motor>, 7> motors_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DmArmController>();
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}