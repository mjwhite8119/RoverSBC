// Author: Martin White

#include <rcutils/cmdline_parser.h>
#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <string>

// #include "rover_node/diff_drive_controller.hpp"
#include "rover_node/rover.hpp"

void help_print()
{
  printf("For rover node : \n");
  printf("rover_node [-i usb_port] [-h]\n");
  printf("options:\n");
  printf("-h : Print this help function.\n");
  printf("-i usb_port: Connected USB port with ESP32.");
}

int main(int argc, char * argv[])
{
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  if (rcutils_cli_option_exist(argv, argv + argc, "-h")) {
    help_print();
    return 0;
  }

  rclcpp::init(argc, argv);

  std::string usb_port = "/dev/ttyUSB1";
  char * cli_options;
  cli_options = rcutils_cli_get_option(argv, argv + argc, "-i");
  if (nullptr != cli_options) {
    usb_port = std::string(cli_options);
  }

  rclcpp::executors::SingleThreadedExecutor executor;

  auto rover = std::make_shared<rover::Rover>(usb_port);
  // auto diff_drive_controller =
  //   std::make_shared<robotis::rover::DiffDriveController>(
  //   rover->get_wheels()->separation,
  //   rover->get_wheels()->radius);

  executor.add_node(rover);
  // executor.add_node(diff_drive_controller);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}