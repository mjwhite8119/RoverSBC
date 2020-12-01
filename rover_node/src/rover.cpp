// Author: Martin White

#include "rover_node/rover.hpp"

#include <memory>
#include <string>

using namespace std::chrono_literals;
using rover::Rover;

Rover::Rover(const std::string & usb_port)
: Node("rover_node", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  RCLCPP_INFO(get_logger(), "Init Rover Node Main");
  node_handle_ = std::shared_ptr<::rclcpp::Node>(this, [](::rclcpp::Node *) {});

  add_motors();
  add_wheels();
  add_sensors();
  add_devices();

  run();
}

Rover::Wheels * Rover::get_wheels()
{
  return &wheels_;
}

Rover::Motors * Rover::get_motors()
{
  return &motors_;
}


void Rover::add_motors()
{
  RCLCPP_INFO(this->get_logger(), "Add Motors");

  this->declare_parameter("motors.profile_acceleration_constant");
  this->declare_parameter("motors.profile_acceleration");

  this->get_parameter_or<float>(
    "motors.profile_acceleration_constant",
    motors_.profile_acceleration_constant,
    214.577);

  this->get_parameter_or<float>(
    "motors.profile_acceleration",
    motors_.profile_acceleration,
    0.0);
}

void Rover::add_wheels()
{
  RCLCPP_INFO(this->get_logger(), "Add Wheels");

  this->declare_parameter("wheels.separation");
  this->declare_parameter("wheels.radius");

  this->get_parameter_or<float>("wheels.separation", wheels_.separation, 0.160);
  this->get_parameter_or<float>("wheels.radius", wheels_.radius, 0.033);
}

void Rover::add_sensors()
{
  RCLCPP_INFO(this->get_logger(), "Add Sensors");  
}

void Rover::add_devices()
{
  RCLCPP_INFO(this->get_logger(), "Add Devices");
}

void Rover::run()
{
  RCLCPP_INFO(this->get_logger(), "Run!");

  publish_timer(std::chrono::milliseconds(50));
  heartbeat_timer(std::chrono::milliseconds(100));
  RCLCPP_INFO(this->get_logger(), "Run! 2");

  parameter_event_callback();
  cmd_vel_callback();
}

void Rover::publish_timer(const std::chrono::milliseconds timeout)
{
  publish_timer_ = this->create_wall_timer(
    timeout,
    [this]() -> void
    {
      rclcpp::Time now = this->now();

      // dxl_sdk_wrapper_->read_data_set();

      // for (const auto & sensor : sensors_) {
      //   sensor->publish(now, dxl_sdk_wrapper_);
      // }
    }
  );
}

void Rover::heartbeat_timer(const std::chrono::milliseconds timeout)
{
  heartbeat_timer_ = this->create_wall_timer(
    timeout,
    [this]() -> void
    {
      static uint8_t count = 0;
      std::string msg;

      // dxl_sdk_wrapper_->set_data_to_device(
      //   extern_control_table.heartbeat.addr,
      //   extern_control_table.heartbeat.length,
      //   &count,
      //   &msg);

      RCLCPP_DEBUG(this->get_logger(), "hearbeat count : %d, msg : %s", count, msg.c_str());

      count++;
    }
  );
}

void Rover::parameter_event_callback()
{
  RCLCPP_INFO(this->get_logger(), "parameter_event_callback");
  priv_parameters_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this);
  RCLCPP_INFO(this->get_logger(), "parameter_event_callback 1");
  while (!priv_parameters_client_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_INFO(this->get_logger(), "parameter_event_callback 2");
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }

    RCLCPP_WARN(this->get_logger(), "service not available, waiting again...");
  }

  auto param_event_callback =
    [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) -> void
    {
      for (const auto & changed_parameter : event->changed_parameters) {
        RCLCPP_DEBUG(
          this->get_logger(),
          "changed parameter name : %s",
          changed_parameter.name.c_str());

        if (changed_parameter.name == "motors.profile_acceleration") {
          std::string sdk_msg;

          motors_.profile_acceleration =
            rclcpp::Parameter::from_parameter_msg(changed_parameter).as_double();

          motors_.profile_acceleration =
            motors_.profile_acceleration / motors_.profile_acceleration_constant;

          union Data {
            int32_t dword[2];
            uint8_t byte[4 * 2];
          } data;

          data.dword[0] = static_cast<int32_t>(motors_.profile_acceleration);
          data.dword[1] = static_cast<int32_t>(motors_.profile_acceleration);

          // uint16_t start_addr = extern_control_table.profile_acceleration_left.addr;
          // uint16_t addr_length =
          //   (extern_control_table.profile_acceleration_right.addr -
          //   extern_control_table.profile_acceleration_left.addr) +
          //   extern_control_table.profile_acceleration_right.length;

          // uint8_t * p_data = &data.byte[0];

          // dxl_sdk_wrapper_->set_data_to_device(start_addr, addr_length, p_data, &sdk_msg);

          RCLCPP_INFO(
            this->get_logger(),
            "changed parameter value : %f [rev/min2] sdk_msg : %s",
            motors_.profile_acceleration,
            sdk_msg.c_str());
        }
      }
    };

  parameter_event_sub_ = priv_parameters_client_->on_parameter_event(param_event_callback);
}

void Rover::cmd_vel_callback()
{
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "cmd_vel",
    qos,
    [this](const geometry_msgs::msg::Twist::SharedPtr msg) -> void
    {
      std::string sdk_msg;

      union Data {
        int32_t dword[6];
        uint8_t byte[4 * 6];
      } data;

      data.dword[0] = static_cast<int32_t>(msg->linear.x * 100);
      data.dword[1] = 0;
      data.dword[2] = 0;
      data.dword[3] = 0;
      data.dword[4] = 0;
      data.dword[5] = static_cast<int32_t>(msg->angular.z * 100);

      // uint16_t start_addr = extern_control_table.cmd_velocity_linear_x.addr;
      // uint16_t addr_length =
      // (extern_control_table.cmd_velocity_angular_z.addr -
      // extern_control_table.cmd_velocity_linear_x.addr) +
      // extern_control_table.cmd_velocity_angular_z.length;

      // uint8_t * p_data = &data.byte[0];

      // dxl_sdk_wrapper_->set_data_to_device(start_addr, addr_length, p_data, &sdk_msg);

      RCLCPP_DEBUG(
        this->get_logger(),
        "lin_vel: %f ang_vel: %f msg : %s", msg->linear.x, msg->angular.z, sdk_msg.c_str());
    }
  );
}
