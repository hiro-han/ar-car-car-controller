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

  serial_ = std::shared_ptr<SerialConnection>(new SerialConnection(this->get_logger()));
  std::string device = declare_parameter("device", "/dev/ttyACM0");
  if (serial_->initialize(device, SerialConnection::kB115200) != 0) {
    RCLCPP_ERROR(this->get_logger(), "Serial initialize error");
  }
  counter_ = 0;

  sleep(3);

  timer_send_ = this->create_wall_timer(1s, std::bind(&SerialTest::send_callback, this));
  // timer_receive_ = this->create_wall_timer(1s, std::bind(&SerialTest::receive_callback, this));
};

void SerialTest::send_callback() {
  CarControlData data;
  data.steer = 0.0f + 0.1f * counter_;
  data.accel = 0.1f + 0.1f * counter_;
  data.camera_direction = 0.2f + 0.1f * counter_;

  RCLCPP_INFO(this->get_logger(), "send: steer = %f, accel = %f, camera_direction = %f", data.steer, data.accel, data.camera_direction);
  try {
    std::vector<uint8_t> array_data(data.bin, data.bin + 12);
    if (!serial_->send(array_data)) {
      RCLCPP_ERROR(this->get_logger(), "Serial send error");
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
  receive_callback();
};

void SerialTest::receive_callback() {
  uint8_t buffer[200];
  // RCLCPP_INFO(this->get_logger(), "recive");

  try {
    size_t size = serial_->receive(true, buffer, 200, '\n');
  // std::cout << "receive = " << buffer << ", size = " << size << std::endl;

    RCLCPP_ERROR(this->get_logger(), "Receive: steer = %s, size = %ld\n", buffer, size);
  } catch (std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "exception receive: %s\n", e.what());
  } catch (...) {
    RCLCPP_ERROR(this->get_logger(), "exception receive: other error");
  }
};

