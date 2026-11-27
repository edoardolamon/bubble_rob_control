/// @file
/// @brief ros2_control hardware plugin for BubbleRob simulated in CoppeliaSim.

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

/// @brief ros2_control hardware for the BubbleRob tutorial robot.
namespace bubble_rob_control
{

/// @brief Connects ros2_control to BubbleRob in CoppeliaSim through the ZeroMQ remote API.
///
/// Exported interfaces, one set per `<joint>` / `<sensor>` of the `<ros2_control>` tag:
/// - `<joint>/velocity` command [rad/s]
/// - `<joint>/position` state [rad], unwrapped (it keeps growing past ±π)
/// - `<joint>/velocity` state [rad/s]
/// - `<sensor>/range` state [m], `+inf` when nothing is detected (REP 117)
///
/// Hardware parameters (`<hardware><param>`):
/// | Name        | Type   | Default     | Unit | Meaning                                      |
/// |-------------|--------|-------------|------|----------------------------------------------|
/// | `host`      | string | `localhost` | –    | host of the CoppeliaSim ZeroMQ server        |
/// | `port`      | int    | `23000`     | –    | port of the CoppeliaSim ZeroMQ server        |
/// | `time_step` | double | `0.01`      | s    | simulation time step, must be 1/update_rate  |
/// | `lockstep`  | bool   | `true`      | –    | advance the simulation by one step per cycle |
///
/// Each `<joint>` and `<sensor>` needs a `sim_path` parameter, the path of the object in the
/// scene (e.g. `/bubbleRob/LeftMotor`).
///
/// @warning With `lockstep` true the simulation advances only when write() is called: if the
///          controller manager stops, the simulation stops too.
class BubbleRobSystem : public hardware_interface::SystemInterface
{
public:
  /// @brief Creates an unconfigured plugin; no connection is opened here.
  BubbleRobSystem();

  /// @brief Destroys the plugin and closes the connection, if any.
  ///
  /// Defined in the .cpp file, where `RemoteAPIClient` is a complete type.
  ~BubbleRobSystem() override;

  /// @brief Reads the hardware and per-joint parameters from the URDF.
  /// @param[in] info parsed `<ros2_control>` tag
  /// @return `SUCCESS`, or `ERROR` if a parameter is malformed, a joint does not have exactly
  ///         one `velocity` command interface, or a joint or sensor has no `sim_path`
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  /// @brief Exports `position` [rad] and `velocity` [rad/s] for each joint and `range` [m]
  ///        for each sensor.
  /// @return the state interfaces, pointing to members of this object
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /// @brief Exports one `velocity` command interface [rad/s] per joint.
  /// @return the command interfaces, pointing to members of this object
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /// @brief Connects to CoppeliaSim and looks up the handle of every `sim_path`.
  /// @return `SUCCESS`, or `ERROR` if CoppeliaSim is not reachable or an object is missing
  /// @pre CoppeliaSim is running with the BubbleRob scene open.
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &) override;

  /// @brief Sets the time step and stepping mode, starts the simulation and reads the
  ///        initial joint positions.
  /// @return `SUCCESS`, or `ERROR` if a remote call fails
  /// @pre The simulation is stopped: CoppeliaSim ignores a time step change while it runs.
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;

  /// @brief Sets the wheel velocities to zero and stops the simulation.
  /// @return always `SUCCESS`; remote errors are only logged
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  /// @brief Closes the connection to CoppeliaSim.
  /// @return always `SUCCESS`
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override;

  /// @brief Reads joint positions and velocities and the proximity sensors.
  /// @return `OK`, or `ERROR` if a remote call fails: the controller manager then
  ///         deactivates the hardware
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;

  /// @brief Sends the velocity commands and, in lockstep mode, advances the simulation by
  ///        one step.
  /// @return `OK`, or `ERROR` if a remote call fails
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  /// @brief State, command and simulator handle of one wheel joint.
  struct Joint
  {
    std::string name;        ///< URDF joint name
    std::string sim_path;    ///< CoppeliaSim object path
    int64_t handle{-1};      ///< CoppeliaSim object handle, -1 until on_configure()
    double last_raw{0.0};    ///< last wrapped angle from the simulator [rad]
    double position{0.0};    ///< unwrapped angle [rad], exported as "<name>/position"
    double velocity{0.0};    ///< [rad/s], exported as "<name>/velocity"
    double command{0.0};     ///< velocity command [rad/s], exported as a command interface
  };

  /// @brief Reading and simulator handle of one proximity sensor.
  struct Sensor
  {
    std::string name;        ///< URDF sensor name
    std::string sim_path;    ///< CoppeliaSim proximity sensor path
    int64_t handle{-1};      ///< CoppeliaSim object handle, -1 until on_configure()
    double range{0.0};       ///< [m], exported as "<name>/range"
  };

  // Filled in on_init() and never resized afterwards: ros2_control keeps
  // pointers to the doubles inside them.
  std::vector<Joint> joints_;     ///< one entry per URDF `<joint>`
  std::vector<Sensor> sensors_;   ///< one entry per URDF `<sensor>`

  std::unique_ptr<RemoteAPIClient> client_;   ///< ZeroMQ connection, set in on_configure()
  rclcpp::Logger logger_{rclcpp::get_logger("BubbleRobSystem")};   ///< plugin logger

  // Set from the URDF <hardware> parameters in on_init();
  // these initial values are only placeholders until then.
  std::string host_{"localhost"};   ///< ZeroMQ server host
  int port_{23000};                 ///< ZeroMQ server port
  double time_step_{0.01};          ///< simulation time step [s]
  bool lockstep_{true};             ///< one simulation step per control cycle
};

}  // namespace bubble_rob_control

#endif  // BUBBLE_ROB_CONTROL__BUBBLE_ROB_SYSTEM_HPP_
