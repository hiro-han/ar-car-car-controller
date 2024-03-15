#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include "ar_car_info/msg/control_info.hpp"
#include "CarControlData.hpp"

class ArCarController : public rclcpp::Node {
 public :
  ArCarController(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ArCarController(const std::string& name_space, const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  virtual ~ArCarController() {};

 private:
  void callback(const ar_car_info::msg::ControlInfo::SharedPtr msg);

  rclcpp::Subscription<ar_car_info::msg::ControlInfo>::SharedPtr control_info_subscription_;

  CarControlData data_;
  SerialConnection serial_;
};
