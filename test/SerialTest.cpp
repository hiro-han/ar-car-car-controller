#include <vector>
#include "SerialTest.hpp"
#include "rclcpp/rclcpp.hpp"
#include "CarControlData.hpp"

using namespace std::chrono_literals;

SerialTest::SerialTest(const rclcpp::NodeOptions& options) : SerialTest("", options) {
  // nop
};

SerialTest::SerialTest(const std::string& name_space, const rclcpp::NodeOptions& options) :
  Node("ar_car_controller", name_space, options) {
  
  send_binary_ = false;

  serial_ = std::shared_ptr<SerialConnection>(new SerialConnection(this->get_logger()));
  std::string device = declare_parameter("device", "/dev/ttyACM0");
  // if (serial_->initialize(device, SerialConnection::kB115200) != 0) {
  if (serial_->initialize(device, SerialConnection::kB9600) != 0) {
    RCLCPP_ERROR(this->get_logger(), "Serial initialize error");
  }
  counter_ = 0;

  sleep(3);

  timer_send_ = this->create_wall_timer(10s, std::bind(&SerialTest::send_callback, this));
  // timer_receive_ = this->create_wall_timer(1s, std::bind(&SerialTest::receive_callback, this));
};

void SerialTest::send_callback() {
  CarControlData data;
  data.steer = 0.0f + 0.1f * counter_;
  data.accel = 0.1f + 0.1f * counter_;
  data.camera_direction = 2.0f + 5.0f * counter_;

  RCLCPP_INFO(this->get_logger(), "send: steer = %f, accel = %f, camera_direction = %f", data.steer, data.accel, data.camera_direction);
  try {
    if (send_binary_) {
      std::vector<uint8_t> array_data(data.bin, data.bin + 12);
      if (!serial_->send(array_data)) {
        RCLCPP_ERROR(this->get_logger(), "Serial send error");
      }
    } else {
      RCLCPP_INFO(this->get_logger(), "step 1");

      std::stringstream ss;
      ss << std::setprecision(2);
      ss << data.accel << "," << data.steer << "," << data.camera_direction;
      RCLCPP_INFO(this->get_logger(), "step 2");

      if (!serial_->send((const uint8_t*)ss.str().c_str(), ss.str().length())) {
        RCLCPP_ERROR(this->get_logger(), "Serial send error");
      }
    }

  } catch (std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "exception send:  %s", e.what());
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "exception send: other error");
  }

  counter_++;
  if (counter_ >= 5) {
    counter_ = 0;
  }
  // receive_callback();
};

void SerialTest::receive_callback() {
  uint8_t buffer[200];

  try {
    size_t size = serial_->receive(true, buffer, 200, 0);

    RCLCPP_INFO(this->get_logger(), "Receive: data = %s, size = %ld", buffer, size);
  } catch (std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "exception receive: %s\n", e.what());
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "exception receive: other error");
  }
};
