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
        
        // 初始化电机对象
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
        
        // 测试电机 每次需确认目标位置是否可达
        // int ID = 1; // 1-7
        //            Id 位置  速度
        test_motor_id(5, -1, 0.1);
        test_motor_id(3, 0.5, 0.1);
        // test_motor_id(6, 1, 0.1);
        // test_motor_id(7, 1, 0.5);

        // RCLCPP_INFO(this->get_logger(), "joint %d : 测试完成！", ID);
    }
    
    // 使能单个电机
    void enable_single_motor(int i){
        motor_control_->enable(*motors_[i]);
    }
    
    // 打印单个电机状态
    void printf_motor(int ID){
        motor_control_->refresh_motor_status(*motors_[ID-1]);
        float current_position = motors_[ID-1]->Get_Position();
        float current_velocity = motors_[ID-1]->Get_Velocity();
        float current_torque = motors_[ID-1]->Get_tau();
        RCLCPP_INFO(this->get_logger(), "Motor ID: %d, Position: %.2f, Velocity: %.2f, Torque: %.2f",
                    motors_[ID-1]->GetSlaveId(), current_position, current_velocity, current_torque);
    }

    // 设置电机 ID 位置/速度
    void test_motor_id(int ID, float position, float velocity){
        // ************单个电机测试*************
        int i = ID;
        // 使能电机
        motor_control_->enable(*motors_[i-1]);

        motor_control_->control_pos_vel(*motors_[i-1], position, velocity);

        // // 延迟5秒打印结果
        // rclcpp::sleep_for(std::chrono::seconds(5)); 
        // printf_motor(ID);
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
    // rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}