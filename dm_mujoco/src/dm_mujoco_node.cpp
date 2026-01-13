#include <mujoco/mujoco.h>
#include <rclcpp/rclcpp.hpp>

class DmMujocoNode : public rclcpp::Node {
public:
  DmMujocoNode() : Node("dm_mujoco_node") {
    RCLCPP_INFO(this->get_logger(), "dm_mujoco node starting...");
    this->declare_parameter<std::string>("model_path", "");
    this->declare_parameter<double>("step_dt", 0.01);

    model_path_ = this->get_parameter("model_path").as_string();
    step_dt_ = this->get_parameter("step_dt").as_double();

    if (model_path_.empty()) {
      RCLCPP_FATAL(this->get_logger(), "Parameter 'model_path' is empty");
      throw std::runtime_error("model_path not set");
    }

    loadModel(model_path_);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(10),
                                     std::bind(&DmMujocoNode::onTimer, this));
  }

private:
  mjModel *model_{nullptr};
  mjData *data_{nullptr};
  std::string model_path_;
  double step_dt_{0.01};
  rclcpp::TimerBase::SharedPtr timer_;

  void onTimer() {
    if (!model_ || !data_) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "MuJoCo model not ready");
      return;
    }

    mj_step(model_, data_);
  }

  void loadModel(const std::string &path) {
    char error[1024] = {0};

    model_ = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
    if (!model_) {
      RCLCPP_FATAL(this->get_logger(), "Failed to load MJCF: %s", error);
      throw std::runtime_error("mj_loadXML failed");
    }

    data_ = mj_makeData(model_);
    if (!data_) {
      mj_deleteModel(model_);
      model_ = nullptr;
      RCLCPP_FATAL(this->get_logger(), "Failed to create mjData");
    }

    RCLCPP_INFO(this->get_logger(), "MuJoCo model loaded: nq=%d nv=%d nu=%d",
                model_->nq, model_->nv, model_->nu);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DmMujocoNode>());
  rclcpp::shutdown();
  return 0;
}
