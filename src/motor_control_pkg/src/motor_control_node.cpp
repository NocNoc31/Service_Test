#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include "rcl_interfaces/msg/parameter.hpp"
#include <vector>
#include <string>
#include <sstream>

class MotorControlNode : public rclcpp::Node
{
public:
  MotorControlNode() : Node("motor_control_node")
  {
    // Declare parameters with default values
    declare_parameter("motor1.target_positions", std::vector<double>{0.0});
    declare_parameter("motor1.speed", 0.0);
    declare_parameter("motor1.accel", 0.0);
    declare_parameter("motor1.min_angle", 0.0);
    declare_parameter("motor1.max_angle", 0.0);
    declare_parameter("motor1.rpm_step", 0.0);
    declare_parameter("motor1.reach_count_max", 0);
    declare_parameter("motor1.repeat", false);

    declare_parameter("motor2.target_positions", std::vector<double>{0.0});
    declare_parameter("motor2.speed", 0.0);
    declare_parameter("motor2.accel", 0.0);
    declare_parameter("motor2.min_angle", 0.0);
    declare_parameter("motor2.max_angle", 0.0);
    declare_parameter("motor2.rpm_step", 0.0);
    declare_parameter("motor2.reach_count_max", 0);
    declare_parameter("motor2.repeat", false);

    // Create services
    services_.push_back(create_service<rcl_interfaces::srv::SetParameters>(
        "/motor1_control_node/set_parameters",
        std::bind(&MotorControlNode::setParametersCallback, this, std::placeholders::_1,
                  std::placeholders::_2, 0)));
    services_.push_back(create_service<rcl_interfaces::srv::SetParameters>(
        "/motor2_control_node/set_parameters",
        std::bind(&MotorControlNode::setParametersCallback, this, std::placeholders::_1,
                  std::placeholders::_2, 1)));

    RCLCPP_INFO(this->get_logger(), "Motor Control Node Started");
  }

private:
  void setParametersCallback(
      const std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> request,
      std::shared_ptr<rcl_interfaces::srv::SetParameters::Response> response,
      int motor_index)
  {
    std::string param_prefix = (motor_index == 0) ? "motor1." : "motor2.";
    
    for (const auto &param : request->parameters) {
      try {
        set_parameter(rclcpp::Parameter(param_prefix + param.name, param.value));
        RCLCPP_INFO(this->get_logger(), "Set %s%s to %s", 
                    param_prefix.c_str(), param.name.c_str(), 
                    parameter_value_to_string(param.value).c_str());
        response->results.push_back(rcl_interfaces::msg::SetParametersResult().set__successful(true));
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to set %s%s: %s", 
                     param_prefix.c_str(), param.name.c_str(), e.what());
        response->results.push_back(rcl_interfaces::msg::SetParametersResult()
                                   .set__successful(false)
                                   .set__reason(e.what()));
      }
    }

    // Log current parameter values
    std::vector<double> temp_positions;
    double speed, accel, min_angle, max_angle, rpm_step;
    int reach_count_max;
    bool repeat;

    get_parameter(param_prefix + "target_positions", temp_positions);
    get_parameter(param_prefix + "speed", speed);
    get_parameter(param_prefix + "accel", accel);
    get_parameter(param_prefix + "min_angle", min_angle);
    get_parameter(param_prefix + "max_angle", max_angle);
    get_parameter(param_prefix + "rpm_step", rpm_step);
    get_parameter(param_prefix + "reach_count_max", reach_count_max);
    get_parameter(param_prefix + "repeat", repeat);

    RCLCPP_INFO(this->get_logger(), "%s Parameters: target_positions=%s, speed=%f, accel=%f, min_angle=%f, max_angle=%f, rpm_step=%f, reach_count_max=%d, repeat=%d",
                param_prefix.c_str(), format_vector(temp_positions).c_str(), 
                speed, accel, min_angle, max_angle, rpm_step, reach_count_max, repeat);
  }

  std::string format_vector(const std::vector<double> &vec) {
    std::string result = "[";
    for (size_t i = 0; i < vec.size(); ++i) {
      result += std::to_string(vec[i]);
      if (i < vec.size() - 1) result += ", ";
    }
    result += "]";
    return result;
  }

  std::string parameter_value_to_string(const rcl_interfaces::msg::ParameterValue &value) {
    std::stringstream ss;
    switch (value.type) {
      case 1: // Bool
        ss << (value.bool_value ? "true" : "false");
        break;
      case 2: // Integer
        ss << value.integer_value;
        break;
      case 3: // Double
        ss << value.double_value;
        break;
      case 7: // Double array
        ss << "[";
        for (size_t i = 0; i < value.double_array_value.size(); ++i) {
          ss << value.double_array_value[i];
          if (i < value.double_array_value.size() - 1) ss << ", ";
        }
        ss << "]";
        break;
      default:
        ss << "unknown";
        break;
    }
    return ss.str();
  }

  std::vector<rclcpp::Service<rcl_interfaces::srv::SetParameters>::SharedPtr> services_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorControlNode>());
  rclcpp::shutdown();
  return 0;
}