#include <rclcpp/rclcpp.hpp>
#include "ArCarController.hpp"

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArCarController>());
  rclcpp::shutdown();
  return 0;
}
