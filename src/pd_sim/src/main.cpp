#include <memory>
#include <string>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "pd_sim/loader.hpp"
#include "pd_sim/visualizer.hpp"

namespace pdsim {

class PDSimulatorNode : public rclcpp::Node {
public:
  PDSimulatorNode() : Node("pd_sim") {
    declareParameters();
    auto &&wheel_model = getRobotModelPath();
    if (!start(wheel_model)) {
      return;
    }
  }

  void declareParameters() {
    rcl_interfaces::msg::ParameterDescriptor param_desc;
    param_desc.description = "Path to gltf file!";
    declare_parameter("robot_model", "", param_desc);
  }

  std::string getRobotModelPath() {
    std::string robot_model = get_parameter("robot_model").as_string();
    RCLCPP_INFO(get_logger(), "Wheel model: %s", robot_model.c_str());
    return robot_model;
  }

  bool start(const std::string &robot_model) {
    tinygltf::Model model;
    if (!loadModel(model, robot_model)) {
      RCLCPP_ERROR(get_logger(), "Failed to load wheel model");
      return false;
    }
    dumpModelInfo(model, get_logger());
    drawLoop(model, 1.f);
    return true;
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace pdsim

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<pdsim::PDSimulatorNode>());
  rclcpp::shutdown();
  return 0;
}