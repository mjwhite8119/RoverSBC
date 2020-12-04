// Author: Martin White

#ifndef ROVER_NODE__DIFF_DRIVE_CONTROLLER_HPP_
#define ROVER_NODE__DIFF_DRIVE_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>

#include <memory>

#include "rover_node/odometry.hpp"

namespace rover
{
class DiffDriveController : public rclcpp::Node
{
public:
  explicit DiffDriveController(const float wheel_seperation, const float wheel_radius);
  virtual ~DiffDriveController() {}

private:
  std::shared_ptr<rclcpp::Node> nh_;
  std::unique_ptr<Odometry> odometry_;
};
}  // namespace rover
#endif  // ROVER_NODE__DIFF_DRIVE_CONTROLLER_HPP_
