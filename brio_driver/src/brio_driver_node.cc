#include "brio_driver/brio_driver.h"
#include "rclcpp/rclcpp.hpp"
#include <memory>

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BrioDriver>());
  rclcpp::shutdown();
  return 0;
}
