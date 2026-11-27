#include <chrono>
#include <cmath>

#include <geometry_msgs/msg/twist_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>

using namespace std::chrono_literals;

class ObstacleAvoider : public rclcpp::Node
{
public:
  ObstacleAvoider()
  : Node("obstacle_avoider")
  {
    forward_speed_  = declare_parameter("forward_speed", 0.28);    // m/s
    reverse_speed_  = declare_parameter("reverse_speed", -0.105);  // m/s
    reverse_turn_   = declare_parameter("reverse_turn", 0.35);     // rad/s
    reverse_time_   = declare_parameter("reverse_time", 3.0);      // s
    sensor_timeout_ = declare_parameter("sensor_timeout", 1.0);    // s

    cmd_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>("cmd_vel", 10);
    sensor_sub_ = create_subscription<sensor_msgs::msg::Range>(
      "range", 10,
      [this](sensor_msgs::msg::Range::ConstSharedPtr msg) {
        last_sensor_ = now();
        // valid readings lie in [min_range, max_range]; +inf means nothing in range
        const bool obstacle = msg->range >= msg->min_range && msg->range <= msg->max_range;
        if (obstacle && !reversing()) {
          reverse_until_ = now() + rclcpp::Duration::from_seconds(reverse_time_);
        }
      });
    timer_ = create_wall_timer(50ms, [this] {publish();});   // 20 Hz
  }

private:
  bool reversing() const {return now() < reverse_until_;}

  void publish()
  {
    geometry_msgs::msg::TwistStamped cmd;
    cmd.header.stamp = now();
    cmd.header.frame_id = "base_link";

    if ((now() - last_sensor_).seconds() > sensor_timeout_) {
      // no sensor data: fail safe, send zero
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "No sensor data, stopping");
    } else if (reversing()) {
      cmd.twist.linear.x = reverse_speed_;
      cmd.twist.angular.z = reverse_turn_;
    } else {
      cmd.twist.linear.x = forward_speed_;
    }
    cmd_pub_->publish(cmd);
  }

  double forward_speed_, reverse_speed_, reverse_turn_, reverse_time_, sensor_timeout_;
  rclcpp::Time last_sensor_{0, 0, RCL_ROS_TIME};
  rclcpp::Time reverse_until_{0, 0, RCL_ROS_TIME};
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr sensor_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObstacleAvoider>());
  rclcpp::shutdown();
  return 0;
}