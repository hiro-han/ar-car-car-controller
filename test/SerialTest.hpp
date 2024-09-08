#pragma once

// #ifndef SERIAL_TEST_H_
// #define SERIAL_TEST_H_

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <memory>
#include "ar_car_info/msg/control_info.hpp"
#include "../src/SerialConnection.hpp"

class SerialTest : public rclcpp::Node {
 public :
  SerialTest(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  SerialTest(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  virtual ~SerialTest() {};

 private:
  void send_callback();
  void receive_callback();

  rclcpp::TimerBase::SharedPtr timer_send_;
  rclcpp::TimerBase::SharedPtr timer_receive_;
  std::shared_ptr<SerialConnection> serial_;

  int counter_;
};

// #endif // SERIAL_TEST_H_
