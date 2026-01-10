#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "dm_ros2_driver/damiao.h"

using namespace damiao;
using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandle = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

class DmArmController : public rclcpp::Node
{
public:
    DmArmController() : Node("dm_arm_controller")
    {
        // 初始化硬件接口（串口和电机控制）
        auto serial = std::make_shared<SerialPort>("/dev/ttyACM0", B921600);
        motor_control_ = std::make_unique<Motor_Control>(serial);

        // 初始化6个电机
        for (int i = 0; i < 6; ++i)
        {
            motors_[i] = std::make_unique<Motor>(DM4310, static_cast<Motor_id>(i + 1), 0x11 + i);
            motor_control_->addMotor(motors_[i].get());
            motor_control_->disable(*motors_[i]);
            motor_control_->switchControlMode(*motors_[i], POS_VEL_MODE);
        }

        // 使能六个电机
        // void enable_single_motor(int ID)
        // {
        //     motor_control_->enable(*motors_[ID - 1]);
        // }
        // 使能所有电机
        for (auto &motor : motors_)
        {
            motor_control_->enable(*motor);
        }

        // 初始化Action Server
        action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
            this,
            "/dm_hand_controller/follow_joint_trajectory",
            std::bind(&DmArmController::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&DmArmController::handle_cancel, this, std::placeholders::_1),
            std::bind(&DmArmController::handle_accepted, this, std::placeholders::_1));

        // 关节名称映射
        joint_id_map_ = {
            {"Joint1", 1},
            {"Joint2", 2},
            {"Joint3", 3},
            {"Joint4", 4},
            {"Joint5", 5},
            {"Joint6", 6},
        };

        RCLCPP_INFO(this->get_logger(), "Action Server Initialized!");
    }

    // 打印六个关节状态
    void print_joint_states(int& num){
        RCLCPP_INFO(this->get_logger(),"第%d组",num);
        num ++;
        for (int i = 0; i < 6; ++i) {
            RCLCPP_INFO(this->get_logger(), "Motor ID: %d, Position: %.2f, Velocity: %.2f, Torque: %.2f",
                motors_[i]->GetSlaveId(), motors_[i]->Get_Position(), motors_[i]->Get_Velocity(), motors_[i]->Get_tau());
        }
    }

private:
    std::unique_ptr<Motor_Control> motor_control_;
    std::array<std::unique_ptr<Motor>, 6> motors_;
    std::map<std::string, uint8_t> joint_id_map_;
    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

    // Action Server回调函数
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received new trajectory goal");
        // 验证关节名称
        for (const auto &name : goal->trajectory.joint_names)
        {
            if (!joint_id_map_.count(name))
            {
                RCLCPP_ERROR(this->get_logger(), "Invalid joint name: %s", name.c_str());
                return rclcpp_action::GoalResponse::REJECT;
            }
        }
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Trajectory execution canceled"); // 轨迹执行取消
        for (auto &motor : motors_){
            motor_control_->disable(*motor); // 停止所有电机
        }
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle)
    {
        std::thread([this, goal_handle](){
            execute_trajectory(goal_handle); 
        }).detach();
    }

    // 轨迹执行逻辑
    void execute_trajectory(const std::shared_ptr<GoalHandle> goal_handle)
    {
        auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
        auto result = std::make_shared<FollowJointTrajectory::Result>();
        static int num = 0;
        const auto start_time = this->now(); // 初始化轨迹开始时间


        try
        {
            // 创建关节名称到电机索引的映射
            std::unordered_map<std::string, size_t> joint_to_index;
            for (size_t i = 0; i < goal->trajectory.joint_names.size(); ++i) {
                const auto &name = goal->trajectory.joint_names[i];
                auto id = joint_id_map_[name];  // 获取关节ID（1-6）
                size_t motor_index = id - 1;    // 转换为电机索引（0-5）
                joint_to_index[name] = motor_index;
            }
            
            // 遍历轨迹点时使用 动态映射
            for (const auto &point : goal->trajectory.points)
            {
                if (goal_handle->is_canceling())
                {
                    result->error_code = result->GOAL_TOLERANCE_VIOLATED;
                    goal_handle->canceled(result);
                    return;
                }

                // 遍历轨迹点的关节状态：控制电机到目标位置
                // for (size_t i = 0; i < point.positions.size(); ++i)
                // {
                //     auto &motor = motors_[i];
                //     float pos = point.positions[i];
                //     float vel = (i < point.velocities.size()) ? point.velocities[i] : 0.0;
                //     motor_control_->control_pos_vel(*motor, pos, vel);
                // }

                // 动态映射：控制电机到目标位置
                for (size_t i = 0; i < point.positions.size(); ++i) {
                    const auto &joint_name = goal->trajectory.joint_names[i];
                    size_t motor_idx = joint_to_index[joint_name];
                    auto &motor = motors_[motor_idx];
                    float pos = point.positions[i];
                    float vel = (i < point.velocities.size()) ? point.velocities[i] : 0.0;
                    motor_control_->control_pos_vel(*motor, pos, vel);
                    
                    // 打印调试信息
                    // if(i ==0){
                    //     RCLCPP_INFO(this->get_logger(),"***********************");
                    //     RCLCPP_INFO(this->get_logger(),"Motor ID: %zu, Position: %.2f, Velocity: %.2f",motor_idx,pos,vel);
                    // }
                }

                // 发布反馈
                feedback->header.stamp = this->now();
                feedback->joint_names = goal->trajectory.joint_names;
                feedback->actual.positions = get_current_positions();
                feedback->desired = point;
                goal_handle->publish_feedback(feedback);
                
                // 轨迹点时间间隔
                auto target_time = start_time + rclcpp::Duration(point.time_from_start);
                auto remaining_time = target_time - this->now();
                if (remaining_time > rclcpp::Duration(0, 0)) {
                    rclcpp::sleep_for(std::chrono::nanoseconds(remaining_time.nanoseconds()));
                }

                // 打印关节状态
                // print_joint_states(num);
            }

            result->error_code = result->SUCCESSFUL;
            goal_handle->succeed(result);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Trajectory execution failed: %s", e.what());
            result->error_code = result->PATH_TOLERANCE_VIOLATED;
            goal_handle->abort(result);
        }
    }

    // 获取当前关节位置
    std::vector<double> get_current_positions()
    {
        std::vector<double> positions;
        for (auto &motor : motors_)
        {
            positions.push_back(motor->Get_Position());
        }
        return positions;
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
