#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <mutex>
#include "ar_car_info/msg/control_info.hpp"
#include "CarControlData.hpp"
#include "SerialConnection.hpp"

class ArCarController : public rclcpp::Node {
 public :
  ArCarController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ArCarController(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  virtual ~ArCarController() {};

 private:
  void callback(const ar_car_info::msg::ControlInfo::SharedPtr msg);
  void sendSerial();
  void sendSerial2();
  void receiveSerial();

  rclcpp::Subscription<ar_car_info::msg::ControlInfo>::SharedPtr control_info_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer2_;
  CarControlData data_;
  SerialConnection serial_;
  std::mutex mutex_;
};
