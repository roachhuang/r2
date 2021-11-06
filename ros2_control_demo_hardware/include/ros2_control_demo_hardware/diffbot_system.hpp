// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROS2_CONTROL_DEMO_HARDWARE__DIFFBOT_SYSTEM_HPP_
#define ROS2_CONTROL_DEMO_HARDWARE__DIFFBOT_SYSTEM_HPP_
#define deg2Rad(angleDegrees) ((angleDegrees) * M_PI / 180.0)
#define rad2Deg(angleRadians) ((angleRadians) * 180.0 / M_PI)
// #include <memory>
// #include <string>
// #include <thread>
#include <vector>
// #include <std_msgs/msg/float32_multi_array.hpp>
#include "hardware_interface/base_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "ros2_control_demo_hardware/visibility_control.h"
// #include "ros2_control_demo_hardware/r2serial.hpp"
#include "BufferedAsyncSerial.hpp"
// #include "ros2_control_demo_hardware/BufferedAsyncSerial.cpp"

// using std::placeholders::_1;
// namespace hw = ros2_control_demo_hardware;
// using namespace std
namespace ros2_control_demo_hardware
{
class DiffBotSystemHardware
: public hardware_interface::BaseInterface<hardware_interface::SystemInterface>
{

  RCLCPP_SHARED_PTR_DEFINITIONS(DiffBotSystemHardware);

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  hardware_interface::return_type configure(const hardware_interface::HardwareInfo & info) override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  hardware_interface::return_type start() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  hardware_interface::return_type stop() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  hardware_interface::return_type read() override;

  ROS2_CONTROL_DEMO_HARDWARE_PUBLIC
  hardware_interface::return_type write() override;

public:
  // std::shared_ptr<SP::SerialPort> port() const;
  /// breif Callback for when serial data are received
  void handleReceivedLine(std::string);  

private:
  // std::shared_ptr<::BufferedAsyncSerial> mySerial;
  ::BufferedAsyncSerial *mySerial{};
  // uart::SimpleSerial *mySerial;

  // std::shared_ptr<::uart::SimpleSerial> mySerial;
  //std::shared_ptr<SP::SerialDriver> serialDrv;
  //std::unique_ptr<SP::SerialPortConfig> m_device_config;
  /*
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
  std::shared_ptr<rclcpp::Node> node2;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr lwheel_deg_sub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rwheel_deg_sub;

  void lwheel_deg_cb(std_msgs::msg::Float32::SharedPtr msg);
  void rwheel_deg_cb(std_msgs::msg::Float32::SharedPtr msg);
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  */

  // Parameters for the DiffBot simulation
  double hw_start_sec_;
  double hw_stop_sec_;

  // Store the command for the simulated robot
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  // Store the wheeled robot position
  double base_x_, base_y_, base_theta_;

  /*
  void lwheel_deg_cb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    // RCLCPP_INFO(this->get_logger(), "/lwheel: '%f'", msg->data);

    //hw_positions_[0] = (double)msg->data;
  }
  void rwheel_deg_cb(const std_msgs::msg::Float32::SharedPtr msg)
  {
    // RCLCPP_INFO(this->get_logger(), "/rwheel: '%f'", msg->data);
    //hw_positions_[1] = (double)msg->data;
  }
  */
};

}  // namespace ros2_control_demo_hardware

#endif  // ROS2_CONTROL_DEMO_HARDWARE__DIFFBOT_SYSTEM_HPP_
