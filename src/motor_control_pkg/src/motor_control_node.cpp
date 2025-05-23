#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/srv/set_parameters.hpp"
#include <vector>
#include <memory>
#include <algorithm>
#include <stdexcept>

class MotorControlNode : public rclcpp::Node {
public:
    MotorControlNode() : Node("motor_control_node") {
        // Khởi tạo giá trị mặc định cho mỗi động cơ
        motor_data_.resize(2); // Hai động cơ
        for (size_t i = 0; i < motor_data_.size(); ++i) {
            motor_data_[i] = {
                {},          // target_positions_
                false,       // target_reached_
                0,           // current_target_index_
                0,           // repeat_counter_
                0,           // speed_
                0,           // accel_
                0.0f,        // rpm_step_
                0            // repeat_
            };
        }

        // Tạo service cho hai động cơ
        services_.push_back(create_service<rcl_interfaces::srv::SetParameters>(
            "/motor1_control_node/set_parameters",
            std::bind(&MotorControlNode::setParametersCallback, this, std::placeholders::_1,
                      std::placeholders::_2, 0)));
        services_.push_back(create_service<rcl_interfaces::srv::SetParameters>(
            "/motor2_control_node/set_parameters",
            std::bind(&MotorControlNode::setParametersCallback, this, std::placeholders::_1,
                      std::placeholders::_2, 1)));

        // Timer để giả lập điều khiển động cơ
        timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                   std::bind(&MotorControlNode::controlLoop, this));

        RCLCPP_INFO(get_logger(), "MotorControlNode initialized with 2 motor services");
    }

private:
    struct MotorData {
        std::vector<float> target_positions_;
        bool target_reached_;
        size_t current_target_index_;
        int repeat_counter_;
        int speed_;
        int accel_;
        float rpm_step_;
        int repeat_;
    };

    std::vector<MotorData> motor_data_;
    std::vector<rclcpp::Service<rcl_interfaces::srv::SetParameters>::SharedPtr> services_;
    rclcpp::TimerBase::SharedPtr timer_;

    void setParametersCallback(
        const std::shared_ptr<rcl_interfaces::srv::SetParameters::Request> request,
        std::shared_ptr<rcl_interfaces::srv::SetParameters::Response> response,
        size_t motor_id) {
        std::string motor_name = (motor_id == 0) ? "Motor 1" : "Motor 2";
        RCLCPP_INFO(get_logger(), "%s: Received set_parameters request", motor_name.c_str());

        if (motor_id >= motor_data_.size()) {
            RCLCPP_ERROR(get_logger(), "%s: Invalid motor index: %zu", motor_name.c_str(), motor_id);
            response->results.resize(request->parameters.size());
            for (size_t i = 0; i < request->parameters.size(); ++i) {
                response->results[i].successful = false;
                response->results[i].reason = "Invalid motor index";
            }
            return;
        }

        response->results.resize(request->parameters.size());
        for (size_t i = 0; i < request->parameters.size(); ++i) {
            const auto& param = request->parameters[i];
            RCLCPP_INFO(get_logger(), "%s: Processing parameter: %s", motor_name.c_str(), param.name.c_str());
            try {
                if (param.name == "target_positions") {
                    auto temp_positions = param.value.double_array_value;
                    motor_data_[motor_id].target_positions_.resize(temp_positions.size());
                    std::transform(temp_positions.begin(), temp_positions.end(),
                                   motor_data_[motor_id].target_positions_.begin(),
                                   [](double x) { return static_cast<float>(x); });
                    motor_data_[motor_id].target_reached_ = false;
                    motor_data_[motor_id].current_target_index_ = 0;
                    motor_data_[motor_id].repeat_counter_ = 0;
                    std::string pos_str;
                    for (double pos : temp_positions) {
                        pos_str += std::to_string(pos) + " ";
                    }
                    RCLCPP_INFO(get_logger(), "%s: Updated target_positions: [%s]", motor_name.c_str(), pos_str.c_str());
                } else if (param.name == "speed") {
                    motor_data_[motor_id].speed_ = param.value.integer_value;
                    RCLCPP_INFO(get_logger(), "%s: Updated speed: %d", motor_name.c_str(), motor_data_[motor_id].speed_);
                } else if (param.name == "accel") {
                    motor_data_[motor_id].accel_ = param.value.integer_value;
                    RCLCPP_INFO(get_logger(), "%s: Updated accel: %d", motor_name.c_str(), motor_data_[motor_id].accel_);
                } else if (param.name == "rpm_step") {
                    motor_data_[motor_id].rpm_step_ = param.value.integer_value;
                    RCLCPP_INFO(get_logger(), "%s: Updated rpm_step: %.2f", motor_name.c_str(), motor_data_[motor_id].rpm_step_);
                } else if (param.name == "repeat") {
                    motor_data_[motor_id].repeat_ = param.value.integer_value;
                    motor_data_[motor_id].repeat_counter_ = 0;
                    RCLCPP_INFO(get_logger(), "%s: Updated repeat: %d", motor_name.c_str(), motor_data_[motor_id].repeat_);
                } else {
                    RCLCPP_WARN(get_logger(), "%s: Unknown parameter: %s", motor_name.c_str(), param.name.c_str());
                }
                response->results[i].successful = true;
                response->results[i].reason = "";
            } catch (const std::exception& e) {
                response->results[i].successful = false;
                response->results[i].reason = e.what();
                RCLCPP_ERROR(get_logger(), "%s: %s", motor_name.c_str(), e.what());
            }
        }
    }

    void controlLoop() {
        for (size_t motor_id = 0; motor_id < motor_data_.size(); ++motor_id) {
            std::string motor_name = (motor_id == 0) ? "Motor 1" : "Motor 2";
            auto& motor = motor_data_[motor_id];
            if (motor.target_positions_.empty()) {
                continue;
            }

            // Giả lập điều khiển động cơ
            if (motor.current_target_index_ < motor.target_positions_.size()) {
                RCLCPP_INFO(get_logger(), "%s: Moving to position %.2f with speed %d, accel %d, rpm_step %.2f",
                            motor_name.c_str(), motor.target_positions_[motor.current_target_index_],
                            motor.speed_, motor.accel_, motor.rpm_step_);
                motor.current_target_index_++;
            }
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<MotorControlNode>());
    } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("motor_control_node"), "Fatal error: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}