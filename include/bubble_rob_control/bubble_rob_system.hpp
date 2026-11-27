#ifndef BUBBLE_ROB_CONTROL__BUBBLE_ROB_SYSTEM_HPP_
#define BUBBLE_ROB_CONTROL__BUBBLE_ROB_SYSTEM_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

class RemoteAPIClient;   // forward declaration: ZeroMQ stays out of the header

namespace bubble_rob_control
{

class BubbleRobSystem : public hardware_interface::SystemInterface
{
public:
  BubbleRobSystem();
  ~BubbleRobSystem() override;   // defined in the .cpp, where RemoteAPIClient is complete

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;

  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  struct Joint
  {
    std::string name;        // URDF joint name
    std::string sim_path;    // CoppeliaSim object path
    int64_t handle{-1};
    double last_raw{0.0};    // last wrapped angle from the simulator
    double position{0.0};    // unwrapped angle, exported as "<name>/position"
    double velocity{0.0};    // exported as "<name>/velocity"
    double command{0.0};     // velocity command, exported as a command interface
  };

  struct Sensor
  {
    std::string name;        // URDF sensor name
    std::string sim_path;    // CoppeliaSim proximity sensor path
    int64_t handle{-1};
    double range{0.0};       // exported as "<name>/range"
  };

  // Filled in on_init() and never resized afterwards: ros2_control keeps
  // pointers to the doubles inside them.
  std::vector<Joint> joints_;
  std::vector<Sensor> sensors_;

  std::unique_ptr<RemoteAPIClient> client_;
  rclcpp::Logger logger_{rclcpp::get_logger("BubbleRobSystem")};

  // Set from the URDF <hardware> parameters in on_init();
  // these initial values are only placeholders until then.
  std::string host_{"localhost"};
  int port_{23000};
  double time_step_{0.01};
  bool lockstep_{true};
};

}  // namespace bubble_rob_control

#endif  // BUBBLE_ROB_CONTROL__BUBBLE_ROB_SYSTEM_HPP_