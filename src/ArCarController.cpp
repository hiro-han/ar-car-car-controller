#include "ArCarController.hpp"
#include <vector>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "SerialConnection.hpp"

using namespace std::chrono_literals;

ArCarController::ArCarController(const rclcpp::NodeOptions& options) : ArCarController("", options) {
  // nop
};

ArCarController::ArCarController(const std::string& name_space, const rclcpp::NodeOptions& options) :
  Node("ar_car_controller", name_space, options) {

  serial_ = std::shared_ptr<SerialConnection>(new SerialConnection(this->get_logger()));
  rcl_interfaces::msg::ParameterDescriptor descriptor;

  speed_limit_rate_ = declare_parameter("speed_limit_rate", 1.0f);
  if (speed_limit_rate_ <= 0.0f || speed_limit_rate_ > 1.0f) {
    RCLCPP_ERROR(this->get_logger(), "speed_limit_rate should be 0.0 < speed_limit_rate <= 1.0");
    speed_limit_rate_ = std::max(std::min(speed_limit_rate_, 1.0f), 0.0f);
  }
  RCLCPP_INFO(this->get_logger(), "speed_limit_rate = %f", speed_limit_rate_);

  std::string device = declare_parameter("device", "/dev/ttyACM0");

  control_info_subscription_ = this->create_subscription<ar_car_info::msg::ControlInfo>(
    "/unity/car_info",
    rclcpp::QoS(10),
    std::bind(&ArCarController::callback, this, std::placeholders::_1)
  );

  auto ret = serial_->initialize(device, SerialConnection::kB9600);
  if (ret != 0) {
    RCLCPP_ERROR(this->get_logger(), "Serial initialize error. code: %d,  device: %s", ret, device.c_str());
  }

  sleep(1);
  timer_ = this->create_wall_timer(0.05s, std::bind(&ArCarController::sendSerial, this));
  // timer_ = this->create_wall_timer(0.1s, std::bind(&ArCarController::receiveSerial, this));
}

void ArCarController::callback(const ar_car_info::msg::ControlInfo::SharedPtr msg){
  mutex_.lock();
  data_.accel = msg->accel;
  data_.steer = msg->steer;
  data_.camera_direction = msg->camera_direction;  
  mutex_.unlock();
}

void ArCarController::sendSerial() {
  mutex_.lock();
  float accel = data_.accel * speed_limit_rate_;
//   std::stringstream ss;
//   ss << std::setprecision(5);
//   ss << accel << "," << data_.steer << "," << data_.camera_direction;
// //  RCLCPP_INFO(this->get_logger(), "send: steer = %f, accel = %f, camera_direction = %f", data_.steer, data_.accel, data_.camera_direction);
//   RCLCPP_INFO(this->get_logger(), "send: data = %s", ss.str().c_str());
//   if (!serial_->send((const uint8_t*)ss.str().c_str(), ss.str().length())) {
//     RCLCPP_ERROR(this->get_logger(), "Serial send error");
//   }

  size_t buffer_size = 15;
  char buffer[buffer_size];
  snprintf(buffer, buffer_size, "%.2f,%.2f,%.2f", accel, data_.steer, data_.camera_direction);
  if (!serial_->send((const uint8_t*)buffer, buffer_size)) {
    RCLCPP_ERROR(this->get_logger(), "Serial send error");
  }
  mutex_.unlock();
};


void ArCarController::receiveSerial() {
  uint8_t buffer[200];
  size_t size = serial_->receive(true, buffer, 200, 0);
  std::cout << "receive = " << buffer << ", size = " << size << std::endl;
};
