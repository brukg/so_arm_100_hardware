#include "so_arm_100_hardware/so_arm_100_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <chrono>
#include <cmath>
#include <limits>
#include <string>
#include <thread>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace so_arm_100_controller
{
SOARM100Interface::SOARM100Interface() 
{
}

SOARM100Interface::~SOARM100Interface()
{
    if (use_serial_) {
        st3215_.end();
    }
}

CallbackReturn SOARM100Interface::on_init(const hardware_interface::HardwareComponentInterfaceParams & params)
{
    CallbackReturn result = hardware_interface::SystemInterface::on_init(params);
    if (result != CallbackReturn::SUCCESS)
    {
        return result;
    }

    use_serial_ = params.hardware_info.hardware_parameters.count("use_serial") ?
        (params.hardware_info.hardware_parameters.at("use_serial") == "true") : false;
    
    serial_port_ = params.hardware_info.hardware_parameters.count("serial_port") ?
        params.hardware_info.hardware_parameters.at("serial_port") : "/dev/ttyUSB0";
    
    serial_baudrate_ = params.hardware_info.hardware_parameters.count("serial_baudrate") ?
        std::stoi(params.hardware_info.hardware_parameters.at("serial_baudrate")) : 1000000;

    servo_speed_ = params.hardware_info.hardware_parameters.count("servo_speed") ?
        std::stoi(params.hardware_info.hardware_parameters.at("servo_speed")) : 2400;

    servo_acceleration_ = params.hardware_info.hardware_parameters.count("servo_acceleration") ?
        std::stoi(params.hardware_info.hardware_parameters.at("servo_acceleration")) : 50;

    // Default control mode: 0=position, 1=velocity, 2=effort/PWM
    default_control_mode_ = params.hardware_info.hardware_parameters.count("control_mode") ?
        std::stoi(params.hardware_info.hardware_parameters.at("control_mode")) : 0;

    size_t num_joints = info_.joints.size();
    position_commands_.resize(num_joints, 0.0);
    position_states_.resize(num_joints, 0.0);
    velocity_commands_.resize(num_joints, 0.0);
    velocity_states_.resize(num_joints, 0.0);
    effort_commands_.resize(num_joints, 0.0);
    effort_states_.resize(num_joints, 0.0);
    active_control_mode_.resize(num_joints, default_control_mode_);
    servo_position_offsets_.resize(num_joints, 0);

    return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SOARM100Interface::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_states_[i]);
        state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]);
        state_interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &effort_states_[i]);
    }
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SOARM100Interface::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &position_commands_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &effort_commands_[i]));
    }
    return command_interfaces;
}

hardware_interface::return_type SOARM100Interface::prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & /*stop_interfaces*/)
{
    // Validate that the requested interfaces are supported
    for (const auto & interface : start_interfaces) {
        if (interface.find("/position") == std::string::npos &&
            interface.find("/velocity") == std::string::npos &&
            interface.find("/effort") == std::string::npos) {
            RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"),
                        "Unsupported command interface: %s", interface.c_str());
            return hardware_interface::return_type::ERROR;
        }
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type SOARM100Interface::perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & /*stop_interfaces*/)
{
    for (const auto & interface : start_interfaces) {
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            if (interface.find(info_.joints[i].name) != std::string::npos) {
                uint8_t new_mode = 0;
                if (interface.find("/effort") != std::string::npos) {
                    new_mode = 2;
                } else if (interface.find("/velocity") != std::string::npos) {
                    new_mode = 1;
                }

                if (active_control_mode_[i] != new_mode) {
                    uint8_t servo_id = static_cast<uint8_t>(i + 1);
                    if (!st3215_.Mode(servo_id, new_mode)) {
                        RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), 
                                    "Failed to set mode %d for servo %d",
                                    new_mode, servo_id);
                        return hardware_interface::return_type::ERROR;
                    }
                    else {
                        active_control_mode_[i] = new_mode;
                        RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"),
                                  "Servo %d switched to mode %d via controller switch",
                                  servo_id, new_mode);
                    }
                }
            }
        }
    }
    return hardware_interface::return_type::OK;
}

CallbackReturn SOARM100Interface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), "Activating so_arm_100 hardware interface...");

    // Read the position offset from EPROM
    auto read_pos_offset = [this](uint8_t servo_id) -> int {
        int raw_pos_offset = -1;
        int err = 0;
        raw_pos_offset = st3215_.readWord(servo_id, SMS_STS_OFS_L);
        if (raw_pos_offset == -1) {
            err = 1;
        }
        if (!err && (raw_pos_offset&(1<<15))) {
            raw_pos_offset = -(raw_pos_offset&~(1<<15));
        }
        return raw_pos_offset;
    };

    if (use_serial_) {
        if(!st3215_.begin(serial_baudrate_, serial_port_.c_str())) {
            RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), "Failed to initialize motors");
            return CallbackReturn::ERROR;
        }

        // Initialize each servo
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            uint8_t servo_id = static_cast<uint8_t>(i + 1);
            
            // First ping the servo
            if (st3215_.Ping(servo_id) == -1) {
                RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), 
                            "No response from servo %d during initialization", servo_id);
                return CallbackReturn::ERROR;
            }
            
            // Set control mode (0=position, 1=velocity, 2=effort/PWM)
            if (!st3215_.Mode(servo_id, default_control_mode_)) {
                RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), 
                            "Failed to set mode for servo %d", servo_id);
                return CallbackReturn::ERROR;
            }

            // Read initial position and set command to match
            if (st3215_.FeedBack(servo_id) != -1) {
                int pos = st3215_.ReadPos(servo_id);
                // calibrate_servo(servo_id, pos);
                position_states_[i] = ticks_to_radians(pos, i);
                position_commands_[i] = position_states_[i];
                RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
                           "Servo %d initialized at position %d", servo_id, pos);
            }

            // Read the position offset from servo EPROM.
            int raw_pos_offset = read_pos_offset(servo_id);
            RCLCPP_DEBUG(rclcpp::get_logger("SOARM100Interface"),
                        "Servo %d: raw_pos_offset=%d", servo_id, raw_pos_offset);
            servo_position_offsets_[i] = raw_pos_offset;

            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
                    "Serial communication initialized on %s", serial_port_.c_str());
    }

    node_ = rclcpp::Node::make_shared("so_arm_100_driver");
    feedback_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "feedback", 10, std::bind(&SOARM100Interface::feedback_callback, this, std::placeholders::_1));
    command_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("command", 10);

    // Add services
    calib_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "record_position",
        std::bind(&SOARM100Interface::calibration_callback, this, 
                  std::placeholders::_1, std::placeholders::_2));
                  
    torque_service_ = node_->create_service<std_srvs::srv::Trigger>(
        "toggle_torque",
        std::bind(&SOARM100Interface::torque_callback, this, 
                  std::placeholders::_1, std::placeholders::_2));

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    spin_thread_ = std::thread([this]() { executor_->spin(); });

    // Load calibration
    std::string calib_file = info_.hardware_parameters.count("calibration_file") ?
        info_.hardware_parameters.at("calibration_file") : "";
        
    if (!calib_file.empty()) {
        if (!load_calibration(calib_file)) {
            RCLCPP_WARN(rclcpp::get_logger("SOARM100Interface"), 
                       "Failed to load calibration file: %s", calib_file.c_str());
        }
    }

    RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), "Hardware interface activated");
    return CallbackReturn::SUCCESS;
}

CallbackReturn SOARM100Interface::on_deactivate(const rclcpp_lifecycle::State &)
{
    if (executor_) {
        executor_->cancel();
    }
    if (spin_thread_.joinable()) {
        spin_thread_.join();
    }
    
    if (use_serial_) {
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            uint8_t servo_id = static_cast<uint8_t>(i + 1);
            st3215_.EnableTorque(servo_id, 0);
        }
    }
    
    RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), "Hardware interface deactivated.");
    return hardware_interface::CallbackReturn::SUCCESS;
}

void SOARM100Interface::feedback_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    last_feedback_msg_ = msg;
}

hardware_interface::return_type SOARM100Interface::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
    if (use_serial_ && torque_enabled_) {  // Only write if torque is enabled
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            uint8_t servo_id = static_cast<uint8_t>(i + 1);

            // Mode is set by perform_command_mode_switch when controllers are switched
            if (active_control_mode_[i] == 2) {
                // Effort/PWM mode: command is percentage (-100 to 100), PWM range is -1000 to 1000
                s16 pwm = static_cast<s16>(std::clamp(-effort_commands_[i], -100.0, 100.0) * 10.0);
                if (!st3215_.RegWritePwm(servo_id, pwm)) {
                    RCLCPP_WARN(rclcpp::get_logger("SOARM100Interface"),
                               "Failed to write PWM to servo %d", servo_id);
                }
            } else if (active_control_mode_[i] == 1) {
                // Velocity mode: convert rad/s to servo speed ticks
                s16 speed_ticks = static_cast<s16>(velocity_commands_[i] * 4096.0 / (2.0 * M_PI));
                if (!st3215_.RegWriteSpe(servo_id, speed_ticks, servo_acceleration_)) {
                    RCLCPP_WARN(rclcpp::get_logger("SOARM100Interface"),
                               "Failed to write velocity to servo %d", servo_id);
                }
            } else {
                // Position mode
                int joint_pos_cmd = radians_to_ticks(position_commands_[i], i);
                RCLCPP_DEBUG(rclcpp::get_logger("SOARM100Interface"),
                           "Servo %d command: %.2f rad -> %d ticks",
                           servo_id, position_commands_[i], joint_pos_cmd);
                if (!st3215_.RegWritePosEx(servo_id, joint_pos_cmd, servo_speed_, servo_acceleration_)) {
                    RCLCPP_WARN(rclcpp::get_logger("SOARM100Interface"),
                               "Failed to write position to servo %d", servo_id);
                }
            }
        }
        st3215_.RegWriteAction();
    }

    if (command_publisher_) {
        sensor_msgs::msg::JointState cmd_msg;
        cmd_msg.header.stamp = node_->now();

        // Establish which control modes are active
        typedef sensor_msgs::msg::JointState::_position_type::value_type value_type;
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            if (active_control_mode_[i] == 2 &&
                cmd_msg.effort.empty()) {
                cmd_msg.effort.resize(info_.joints.size(),
                    std::numeric_limits<value_type>::quiet_NaN());
            } else if (active_control_mode_[i] == 1 &&
                       cmd_msg.velocity.empty()) {
                cmd_msg.velocity.resize(info_.joints.size(),
                    std::numeric_limits<value_type>::quiet_NaN());
            } else {
                cmd_msg.position.resize(info_.joints.size(),
                    std::numeric_limits<value_type>::quiet_NaN());
            }
        }

        // Populate commands
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            cmd_msg.name.push_back(info_.joints[i].name);
            if (active_control_mode_[i] == 2) {
                cmd_msg.effort[i] = effort_commands_[i];
            } else if (active_control_mode_[i] == 1) {
                cmd_msg.velocity[i] = velocity_commands_[i];
            } else {
                cmd_msg.position[i] = position_commands_[i];
            }
        }

        command_publisher_->publish(cmd_msg);
    }

    return hardware_interface::return_type::OK;
}

hardware_interface::return_type SOARM100Interface::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Apply position offset to the raw position
  auto apply_pos_offset = [](int pos, int pos_offset) -> int {
    int homing_offset = pos_offset > 2048 ? 2048 - pos_offset : pos_offset;
    int offset_pos = (pos - homing_offset) % 4096;
    return offset_pos;
  };

  if (use_serial_) {
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            uint8_t servo_id = static_cast<uint8_t>(i + 1);

            if (!torque_enabled_) {
                // When torque is disabled, only try to read position
                int raw_pos = st3215_.ReadPos(servo_id);
                if (raw_pos != -1) {
                    // Apply position offset in velocity and effort modes
                    int raw_pos_offset = servo_position_offsets_[i];
                    if (active_control_mode_[i] != 0 && raw_pos_offset != 0) {
                        raw_pos = apply_pos_offset(raw_pos, raw_pos_offset);
                    }
                    position_states_[i] = ticks_to_radians(raw_pos, i);
                }
                continue;  // Skip other reads
            }

            // FeedBack() reads all registers (pos, speed, load, voltage, temp, current)
            // in one serial transaction. Use -1 for subsequent reads to use cached data.
            if (st3215_.FeedBack(servo_id) != -1) {
                int raw_pos = st3215_.ReadPos(-1);
                // Apply position offset in velocity and effort modes
                int raw_pos_offset = servo_position_offsets_[i];
                if (active_control_mode_[i] != 0 && raw_pos_offset != 0) {
                    raw_pos = apply_pos_offset(raw_pos, raw_pos_offset);
                }
                position_states_[i] = ticks_to_radians(raw_pos, i);

                velocity_states_[i] = st3215_.ReadSpeed(-1) * 2.0 * M_PI / 4096.0;
                effort_states_[i] = st3215_.ReadLoad(-1) / 10.0;

                double temperature = st3215_.ReadTemper(-1);
                double voltage = st3215_.ReadVoltage(-1) / 10.0;
                double current = st3215_.ReadCurrent(-1) * 6.5 / 1000.0;

                RCLCPP_DEBUG(rclcpp::get_logger("SOARM100Interface"),
                            "Servo %d: raw_pos=%d (%.2f rad) vel=%.2f effort=%.2f temp=%.1f V=%.1f I=%.3f",
                            servo_id, raw_pos, position_states_[i], velocity_states_[i],
                            effort_states_[i], temperature, voltage, current);
            } else {
                RCLCPP_WARN(rclcpp::get_logger("SOARM100Interface"), 
                           "Failed to read feedback from servo %d", servo_id);
            }
        }
    }
    else {
        sensor_msgs::msg::JointState::SharedPtr feedback_copy;
        {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            feedback_copy = last_feedback_msg_;
        }

        if (feedback_copy) {
            for (size_t i = 0; i < info_.joints.size(); ++i) {
                auto it = std::find(feedback_copy->name.begin(), feedback_copy->name.end(), info_.joints[i].name);
                if (it != feedback_copy->name.end()) {
                    size_t idx = std::distance(feedback_copy->name.begin(), it);
                    if (idx < feedback_copy->position.size()) {
                        position_states_[i] = ticks_to_radians(feedback_copy->position[idx], i);
                    }
                }
            }
        }
    }

    return hardware_interface::return_type::OK;
}

void SOARM100Interface::calibrate_servo(uint8_t servo_id, int current_pos) 
{
    size_t idx = servo_id - 1;
    // Calculate offset from current position to expected zero
    int offset = current_pos - zero_positions_[idx];
    RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
               "Servo %d: current=%d, zero=%d, offset=%d", 
               servo_id, current_pos, zero_positions_[idx], offset);
}

double SOARM100Interface::ticks_to_radians(int ticks, size_t servo_idx) 
{
    const std::string& joint_name = info_.joints[servo_idx].name;
    
    if (joint_calibration_.count(joint_name) > 0) {
        const auto& calib = joint_calibration_[joint_name];
        // Convert to normalized position first (0 to 1)
        double normalized = (double)(ticks - calib.min_ticks) / calib.range_ticks;
        // Then convert to radians (-π to π)
        return (normalized * 2.0 - 1.0) * M_PI;
    }
    
    // Fallback to default calibration
    return servo_directions_[servo_idx] * 
           (ticks - zero_positions_[servo_idx]) * 2 * M_PI / 4096.0;
}

int SOARM100Interface::radians_to_ticks(double radians, size_t servo_idx) 
{
    const std::string& joint_name = info_.joints[servo_idx].name;
    
    if (joint_calibration_.count(joint_name) > 0) {
        const auto& calib = joint_calibration_[joint_name];
        // Convert from radians (-π to π) to normalized position (0 to 1)
        double normalized = (radians / M_PI + 1.0) / 2.0;
        // Then convert to ticks
        return calib.min_ticks + (int)(normalized * calib.range_ticks);
    }
    
    // Fallback to default calibration
    return zero_positions_[servo_idx] + 
           servo_directions_[servo_idx] * (int)(radians * 4096.0 / (2 * M_PI));
}

void SOARM100Interface::record_current_position() 
{
    std::stringstream ss;
    ss << "{";  // Start with just a curly brace
    
    bool first = true;  // To handle commas between entries
    for (size_t i = 0; i < info_.joints.size(); ++i) {
        uint8_t servo_id = static_cast<uint8_t>(i + 1);
        
        // Add delay between reads
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
        // Try multiple times to read the servo
        int pos = -1;
        for (int retry = 0; retry < 3 && pos == -1; retry++) {
            st3215_.FeedBack(servo_id);
            pos = st3215_.ReadPos(servo_id);
            if (pos == -1) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
        }
        
        if (!first) {
            ss << ",";
        }
        first = false;
        
        ss << "\"" << info_.joints[i].name << "\": {"
           << "\"ticks\": " << (pos != -1 ? pos : 0) << ","
           << "\"speed\": " << st3215_.ReadSpeed(servo_id) << ","
           << "\"load\": " << st3215_.ReadLoad(servo_id)
           << "}";
    }
    ss << "}";  // Close the JSON object
    
    last_calibration_data_ = ss.str();
    RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
                "Recorded positions: %s", last_calibration_data_.c_str());
}

void SOARM100Interface::calibration_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    record_current_position();
    response->success = true;
    response->message = last_calibration_data_;
}

void SOARM100Interface::set_torque_enable(bool enable) 
{
    if (use_serial_) {
        // First set all servos
        for (size_t i = 0; i < info_.joints.size(); ++i) {
            uint8_t servo_id = static_cast<uint8_t>(i + 1);
            
            if (!enable) {
                // When disabling:
                // 1. Set to idle mode first
                st3215_.Mode(servo_id, 2);  // Mode 2 = idle
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                
                // 2. Disable torque
                st3215_.EnableTorque(servo_id, 0);
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
                
                // 3. Double check it's disabled
                st3215_.EnableTorque(servo_id, 0);
            } else {
                // When enabling:
                // 1. Set position mode
                st3215_.Mode(servo_id, 0);  // Mode 0 = position
                active_control_mode_[i] = 0;
                velocity_commands_[i] = 0.0;
                effort_commands_[i] = 0.0;
                std::this_thread::sleep_for(std::chrono::milliseconds(10));

                // 2. Enable torque
                st3215_.EnableTorque(servo_id, 1);
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        // Wait a bit to ensure commands are processed
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        
        // Update state after all servos are set
        torque_enabled_ = enable;
        
        RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
                    "Torque %s for all servos", enable ? "enabled" : "disabled");
    }
}

void SOARM100Interface::torque_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    bool new_state = !torque_enabled_;
    
    // Set response before changing state
    response->success = true;
    response->message = std::string("Torque ") + (new_state ? "enabled" : "disabled");
    
    // Change state after setting response
    set_torque_enable(new_state);
    
    RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
                "Torque service called, response: %s", response->message.c_str());
}

bool SOARM100Interface::load_calibration(const std::string& filepath) 
{
    try {
        YAML::Node config = YAML::LoadFile(filepath);
        auto joints = config["joints"];
        if (!joints) {
            RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), 
                        "No joints section in calibration file");
            return false;
        }

        for (const auto& joint : joints) {
            std::string name = joint.first.as<std::string>();
            const auto& data = joint.second;
            
            if (!data["min"] || !data["center"] || !data["max"]) {
                RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), 
                            "Missing calibration data for joint %s", name.c_str());
                continue;
            }

            JointCalibration calib;
            calib.min_ticks = data["min"]["ticks"].as<int>();
            calib.center_ticks = data["center"]["ticks"].as<int>();
            calib.max_ticks = data["max"]["ticks"].as<int>();
            calib.range_ticks = calib.max_ticks - calib.min_ticks;
            
            joint_calibration_[name] = calib;
            
            RCLCPP_INFO(rclcpp::get_logger("SOARM100Interface"), 
                       "Loaded calibration for %s: min=%d, center=%d, max=%d", 
                       name.c_str(), calib.min_ticks, calib.center_ticks, calib.max_ticks);
        }
        return true;
    } catch (const YAML::Exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("SOARM100Interface"), 
                    "Failed to load calibration: %s", e.what());
        return false;
    }
}

double SOARM100Interface::normalize_position(const std::string& joint_name, int ticks) 
{
    if (joint_calibration_.count(joint_name) == 0) {
        return 0.0;
    }
    
    const auto& calib = joint_calibration_[joint_name];
    double normalized = (ticks - calib.min_ticks) / calib.range_ticks;
    return std::clamp(normalized, 0.0, 1.0);
}

}  // namespace so_arm_100_controller

PLUGINLIB_EXPORT_CLASS(so_arm_100_controller::SOARM100Interface, hardware_interface::SystemInterface)

