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

#include <boost/algorithm/string.hpp>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <memory>
#include <vector>

#include <thread>
// #include "BufferedAsyncSerial.hpp"

// #include <rclcpp/rclcpp.hpp>
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "ros2_control_demo_hardware/diffbot_system.hpp"

// #include "ros2_control_demo_hardware/serial_driver.hpp"

// static constexpr const char * dev_name = "/dev/ttyUSB0";
// static constexpr uint32_t baud = 115200;

namespace ros2_control_demo_hardware
{
hardware_interface::return_type DiffBotSystemHardware::configure(
  const hardware_interface::HardwareInfo & info)
{
  base_x_ = 0.0;
  base_y_ = 0.0;
  base_theta_ = 0.0;

  if (configure_default(info) != hardware_interface::return_type::OK)
  {
    return hardware_interface::return_type::ERROR;
  }

  /*
  m_device_config = std::make_unique<SP::SerialPortConfig>(
    115200, SP::FlowControl::NONE, SP::Parity::NONE, SP::StopBits::ONE);
  */

  hw_start_sec_ = stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  hw_positions_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocities_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DiffBotSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("diffbot"), "Joint '%s' has %d command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("diffbot"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("diffbot"), "Joint '%s' has %d state interface. 2 expected.",
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("diffbot"),
        "Joint '%s' have '%s' as first state interface. '%s' and '%s' expected.",
        joint.name.c_str(), joint.state_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::return_type::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("diffbot"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::return_type::ERROR;
    }
  }

  status_ = hardware_interface::status::CONFIGURED;
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> DiffBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DiffBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type DiffBotSystemHardware::start()
{
  RCLCPP_INFO(rclcpp::get_logger("diffbot"), "Starting ...please wait...");

  for (auto i = 0; i <= hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(rclcpp::get_logger("diffbot"), "%.1f seconds left...", hw_start_sec_ - i);
  }

  // set some default values
  for (auto i = 0u; i < hw_positions_.size(); i++)
  {
    if (std::isnan(hw_positions_[i]))
    {
      hw_positions_[i] = 0;
      hw_velocities_[i] = 0;
      hw_commands_[i] = 0;
    }
  }

  try
  {
    //Return immediately. String is written *after* the function returns,
    //in a separate thread.
    // using namespace std;
    // mySerial = new ::BufferedAsyncSerial("/dev/ttyUSB0", 115200);  // create a comport obj
    mySerial = new uart::SimpleSerial("/dev/ttyUSB0", 115200);  // create a comport obj

    //Simulate doing something else while the serial device replies.
    //When the serial device replies, the second thread stores the received
    //data in a buffer.
    // std::this_thread::sleep_for(std::chrono::seconds(2));
  }
  catch (boost::system::system_error & e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("serial"), "%s", e.what());
    status_ = hardware_interface::status::UNKNOWN;
    return hardware_interface::return_type::ERROR;
  }

  status_ = hardware_interface::status::STARTED;
  /*
  if (!rclcpp::ok())
  {   
    rclcpp::init(1, NULL); // , rclcpp::InitOptions());
  }
  node2 = rclcpp::Node::make_shared("hw");
  this->sub_ = node2->create_subscription<std_msgs::msg::String>(
    "chatter", rclcpp::QoS(10).best_effort(),
    std::bind(&DiffBotSystemHardware::callback, this, std::placeholders::_1));
  // rclcpp::spin(thissafdadsfsfsdaf->node2);
  // rclcpp::spin(std::make_shared<DiffBotSystemHardware>());

  // Spin up the queue helper thread.
  // this->rosQueueThread = std::thread(std::bind(&DiffBotSystemHardware::rosQueueThread, this));
*/
  RCLCPP_INFO(rclcpp::get_logger("diffbot"), "System Successfully started!");

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DiffBotSystemHardware::stop()
{
  RCLCPP_INFO(rclcpp::get_logger("diffbot"), "Stopping ...please wait...");

  for (auto i = 0; i <= hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(rclcpp::get_logger("diffbot"), "%.1f seconds left...", hw_stop_sec_ - i);
  }
  delete mySerial;
  ros2_control_demo_hardware::DiffBotSystemHardware::

    status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(rclcpp::get_logger("diffbot"), "System successfully stopped!");
  rclcpp::shutdown();
  return hardware_interface::return_type::OK;
}

void ros2_control_demo_hardware::DiffBotSystemHardware::handleReceivedLine(std::string line)
{
  std::vector<std::string> lineParts;

  boost::split(lineParts, line, boost::is_any_of("\t"));
  // RCLCPP_INFO(rclcpp::get_logger("serial"), "PARSE: %s", lineParts[0]);
  if (lineParts[0] == "e")
  {
    hw_positions_[0] = std::stod(lineParts[1]);  // left
    hw_positions_[1] = std::stod(lineParts[2]);  // right
    RCLCPP_INFO(rclcpp::get_logger("serial"), "l: %s r: %s", lineParts[1].c_str(), lineParts[2].c_str());
  }
  /*
            elif (lineParts[0] == 'v'):
                self._rwheel_vel_value = float(lineParts[1])
                self._lwheel_vel_value = float(lineParts[2])                                        
            else:
                
*/
}

hardware_interface::return_type DiffBotSystemHardware::read()
{
  RCLCPP_INFO(rclcpp::get_logger("diffbot"), "Reading...");
  // handleReceivedLine(mySerial->readStringUntil("\n"));
  handleReceivedLine(mySerial->readLine());

  double radius = 0.02;  // radius of the wheels
  double dist_w = 0.1;   // distance between the wheels
  double dt = 0.01;      // Control period

  // RCLCPP_INFO(rclcpp::get_logger("serial"), "ReadLine: %s", mySerial->readLine());

  for (uint i = 0; i < hw_commands_.size(); i++)
  {
    // Simulate DiffBot wheels's movement as a first-order system
    // Update the joint status: this is a revolute joint without any limit.
    // Simply integrates
    hw_positions_[i] = hw_positions_[i] + dt * hw_commands_[i];
    hw_velocities_[i] = hw_commands_[i];

    RCLCPP_INFO(
      rclcpp::get_logger("diffbot"), "Got position state %.5f and velocity state %.5f for '%s'!",
      hw_positions_[i], hw_velocities_[i], info_.joints[i].name.c_str());
  }

  // Update the free-flyer, i.e. the base notation using the classical
  // wheel differentiable kinematics
  double base_dx = 0.5 * radius * (hw_commands_[0] + hw_commands_[1]) * cos(base_theta_);
  double base_dy = 0.5 * radius * (hw_commands_[0] + hw_commands_[1]) * sin(base_theta_);
  double base_dtheta = radius * (hw_commands_[0] - hw_commands_[1]) / dist_w;
  base_x_ += base_dx * dt;
  base_y_ += base_dy * dt;
  base_theta_ += base_dtheta * dt;

  RCLCPP_INFO(
    rclcpp::get_logger("diffbot"), "Joints successfully read! (%.5f,%.5f,%.5f)", base_x_, base_y_,
    base_theta_);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ros2_control_demo_hardware::DiffBotSystemHardware::write()
{
  RCLCPP_INFO(rclcpp::get_logger("diffbot"), "Writing...");

  for (auto i = 0u; i < hw_commands_.size(); i++)
  {
    // Simulate sending commands to the hardware
    RCLCPP_INFO(
      rclcpp::get_logger("diffbot"), "Got command %.5f for '%s'!", hw_commands_[i],
      info_.joints[i].name.c_str());
  }
  /*
  char values[]={0xde,0xad,0xbe,0xef};
  serial.write(values,sizeof(values));
*/
  // Send the motor command
  // serial_port_->write_frame(hw_commands_, 6);
  RCLCPP_INFO(rclcpp::get_logger("diffbot"), "Joints successfully written!");

  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_hardware::DiffBotSystemHardware, hardware_interface::SystemInterface)
