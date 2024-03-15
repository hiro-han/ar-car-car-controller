#include <rclcpp/rclcpp.hpp>
#include "SerialTest.hpp"

int main(int argc, char **argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SerialTest>());
  rclcpp::shutdown();
  return 0;
}
