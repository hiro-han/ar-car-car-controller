#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <mutex>
#include <memory>
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
  void receiveSerial();

  rclcpp::Subscription<ar_car_info::msg::ControlInfo>::SharedPtr control_info_subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  CarControlData data_;
  std::shared_ptr<SerialConnection> serial_;
  // SerialConnection serial_;
  std::mutex mutex_;
  float speed_limit_rate_;
};
