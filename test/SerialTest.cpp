#include "SerialTest.hpp"
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "CarControlData.hpp"
#include "SerialPacket.hpp"

using namespace std::chrono_literals;

SerialTest::SerialTest(const rclcpp::NodeOptions& options) : SerialTest("", options) {
  // nop
};

SerialTest::SerialTest(const std::string& name_space, const rclcpp::NodeOptions& options) :
  Node("ar_car_controller", name_space, options) {

  std::string device = "/dev/ttyACM0";
  if (!serial_.initialize(device, SerialConnection::kB115200)) {
    RCLCPP_ERROR(this->get_logger(), "Serial initialize error");
  }
  counter_ = 0;

  timer_ = this->create_wall_timer(1s, std::bind(&SerialTest::callback, this));
};

void SerialTest::callback() {
  CarControlData data;
  data.steer = 1.1f * counter_;
  data.accel = 2.2f * counter_;
  data.camera_direction = 3.3f * counter_;

  RCLCPP_INFO(this->get_logger(), "Receive: steer = %f, accel = %f, camera_direction = %f", data.steer, data.accel, data.camera_direction);

  std::vector<uint8_t> array_data(data.bin, data.bin + 12);

  std::vector<uint8_t> packet = EncodePacket(array_data);
  if (!serial_.send(packet)) {
    RCLCPP_ERROR(this->get_logger(), "Serial send error");
  }
  counter_++;
};

