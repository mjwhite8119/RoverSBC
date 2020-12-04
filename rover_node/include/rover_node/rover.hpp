// Author: Martin White

#ifndef ROVER_NODE__ROVER_HPP_
#define ROVER_NODE__ROVER_HPP_

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>
// #include <rover_msgs/msg/sensor_state.hpp>

#include <array>
#include <chrono>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <queue>

// #include "rover_node/control_table.hpp"
// #include "rover_node/dynamixel_sdk_wrapper.hpp"
#include "rover_node/odometry.hpp"

// #include "rover_node/devices/devices.hpp"
// #include "rover_node/devices/motor_power.hpp"
// #include "rover_node/devices/reset.hpp"
// #include "rover_node/devices/sound.hpp"

// #include "rover_node/sensors/battery_state.hpp"
// #include "rover_node/sensors/imu.hpp"
// #include "rover_node/sensors/joint_state.hpp"
// #include "rover_node/sensors/sensor_state.hpp"
// #include "rover_node/sensors/sensors.hpp"

namespace rover
{
// extern const ControlTable extern_control_table;
class Rover : public rclcpp::Node
{
public:
  typedef struct
  {
    float separation;
    float radius;
  } Wheels;

  typedef struct
  {
    float profile_acceleration_constant;
    float profile_acceleration;
  } Motors;

  // Constructor
  explicit Rover(const std::string & usb_port);

  // Destructor
  virtual ~Rover() {}

  Wheels * get_wheels();
  Motors * get_motors();

private:
  // void init_dynamixel_sdk_wrapper(const std::string & usb_port);
  // void check_device_status();

  void add_sensors();
  void add_devices();
  void add_motors();
  void add_wheels();

  void run();

  void publish_timer(const std::chrono::milliseconds timeout);
  void heartbeat_timer(const std::chrono::milliseconds timeout);

  void cmd_vel_callback();
  void parameter_event_callback();

  Wheels wheels_;
  Motors motors_;

  // std::list<sensors::Sensors *> sensors_;
  // std::map<std::string, devices::Devices *> devices_;

  std::unique_ptr<Odometry> odom_;

  rclcpp::Node::SharedPtr node_handle_;

  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr heartbeat_timer_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  rclcpp::AsyncParametersClient::SharedPtr priv_parameters_client_;
  rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr parameter_event_sub_;
};
}  // namespace rover

#endif  // ROVER_NODE__ROVER_HPP_
