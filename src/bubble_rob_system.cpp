#include "bubble_rob_control/bubble_rob_system.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "RemoteAPIClient.h"

namespace bubble_rob_control
{
using hardware_interface::CallbackReturn;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;
using hardware_interface::return_type;

BubbleRobSystem::BubbleRobSystem() = default;
BubbleRobSystem::~BubbleRobSystem() = default;

CallbackReturn BubbleRobSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const auto & hw = info_.hardware_parameters;
  const auto param = [&hw](const std::string & key, const std::string & fallback) {
      const auto it = hw.find(key);
      return it == hw.end() ? fallback : it->second;
    };
  try {
    host_ = param("host", "localhost");
    port_ = std::stoi(param("port", "23000"));
    time_step_ = std::stod(param("time_step", "0.01"));
    lockstep_ = param("lockstep", "true") == "true";
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger_, "Bad hardware parameter: %s", e.what());
    return CallbackReturn::ERROR;
  }

  for (const auto & j : info_.joints) {
    if (j.command_interfaces.size() != 1 || j.command_interfaces[0].name != HW_IF_VELOCITY) {
      RCLCPP_FATAL(logger_, "Joint '%s' needs exactly one velocity command interface",
        j.name.c_str());
      return CallbackReturn::ERROR;
    }
    const auto path = j.parameters.find("sim_path");
    if (path == j.parameters.end()) {
      RCLCPP_FATAL(logger_, "Joint '%s' has no 'sim_path' parameter", j.name.c_str());
      return CallbackReturn::ERROR;
    }
    joints_.push_back(Joint{j.name, path->second});
  }

  for (const auto & s : info_.sensors) {
    const auto path = s.parameters.find("sim_path");
    if (path == s.parameters.end()) {
      RCLCPP_FATAL(logger_, "Sensor '%s' has no 'sim_path' parameter", s.name.c_str());
      return CallbackReturn::ERROR;
    }
    sensors_.push_back(Sensor{s.name, path->second});
  }
  return CallbackReturn::SUCCESS;
}

// Hand ros2_control pointers to our own variables: read() updates them in place,
// and controllers read them through these pointers.
std::vector<hardware_interface::StateInterface> BubbleRobSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (auto & j : joints_) {
    out.emplace_back(j.name, HW_IF_POSITION, &j.position);
    out.emplace_back(j.name, HW_IF_VELOCITY, &j.velocity);
  }
  for (auto & s : sensors_) {
    out.emplace_back(s.name, "range", &s.range);
  }
  return out;
}

// The controller writes into j.command; write() sends it to CoppeliaSim.
std::vector<hardware_interface::CommandInterface> BubbleRobSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (auto & j : joints_) {
    out.emplace_back(j.name, HW_IF_VELOCITY, &j.command);
  }
  return out;
}

CallbackReturn BubbleRobSystem::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    client_ = std::make_unique<RemoteAPIClient>(host_, port_);
    auto sim = client_->getObject().sim();
    // getObject throws if a path is not in the scene, so a wrong scene fails here, with the
    // path in the message, and not later in read()
    for (auto & j : joints_) {
      j.handle = sim.getObject(j.sim_path);
    }
    for (auto & s : sensors_) {
      s.handle = sim.getObject(s.sim_path);
    }
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger_, "Cannot reach CoppeliaSim at %s:%d: %s",
      host_.c_str(), port_, e.what());
    return CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger_, "Connected to CoppeliaSim at %s:%d", host_.c_str(), port_);
  return CallbackReturn::SUCCESS;
}

CallbackReturn BubbleRobSystem::on_activate(const rclcpp_lifecycle::State &)
{
  try {
    auto sim = client_->getObject().sim();
    sim.setFloatParam(sim.floatparam_simulation_time_step, time_step_);  // only while stopped
    sim.setStepping(lockstep_);
    sim.startSimulation();
    // start the unwrapping from the current angle, and from rest: a controller that is
    // activated later must not see a stale command
    for (auto & j : joints_) {
      j.last_raw = sim.getJointPosition(j.handle);
      j.position = j.last_raw;
      j.command = 0.0;
    }
  } catch (const std::exception & e) {
    RCLCPP_FATAL(logger_, "Activation failed: %s", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn BubbleRobSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  try {
    auto sim = client_->getObject().sim();
    for (const auto & j : joints_) {
      sim.setJointTargetVelocity(j.handle, 0.0);
    }
    sim.stopSimulation();
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "Deactivation: %s", e.what());
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn BubbleRobSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  client_.reset();
  return CallbackReturn::SUCCESS;
}

return_type BubbleRobSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  try {
    auto sim = client_->getObject().sim();
    for (auto & j : joints_) {
      const double raw = sim.getJointPosition(j.handle);
      j.position += std::remainder(raw - j.last_raw, 2.0 * M_PI);  // unwrap the ±π jump
      j.last_raw = raw;
      j.velocity = sim.getJointVelocity(j.handle);
    }
    for (auto & s : sensors_) {
      // distance when something is detected, otherwise +inf:
      // the ROS convention (REP 117) for "nothing in range"
      auto [detected, distance, point, object, normal] = sim.readProximitySensor(s.handle);
      s.range = detected > 0 ? distance : std::numeric_limits<double>::infinity();
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "read() failed: %s", e.what());
    return return_type::ERROR;   // controller_manager deactivates the hardware
  }
  return return_type::OK;
}

return_type BubbleRobSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  try {
    auto sim = client_->getObject().sim();
    for (const auto & j : joints_) {
      sim.setJointTargetVelocity(j.handle, j.command);
    }
    if (lockstep_) {
      sim.step();   // one control cycle == one physics step
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(logger_, "write() failed: %s", e.what());
    return return_type::ERROR;
  }
  return return_type::OK;
}

}  // namespace bubble_rob_control

PLUGINLIB_EXPORT_CLASS(bubble_rob_control::BubbleRobSystem, hardware_interface::SystemInterface)