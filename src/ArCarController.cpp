#include "ArCarController.hpp"
#include <vector>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "SerialPacket.hpp"
#include "SerialConnection.hpp"

using namespace std::chrono_literals;

ArCarController::ArCarController(const rclcpp::NodeOptions& options) : ArCarController("", options) {
  // nop
};

ArCarController::ArCarController(const std::string& name_space, const rclcpp::NodeOptions& options) :
  Node("ar_car_controller", name_space, options) {

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

  auto ret = serial_.initialize(device, SerialConnection::kB9600);
  if (ret != 0) {
    RCLCPP_ERROR(this->get_logger(), "Serial initialize error. code: %d,  device: %s", ret, device.c_str());
  }

  sleep(1);
  timer_ = this->create_wall_timer(0.05s, std::bind(&ArCarController::sendSerial2, this));
  // timer2_ = this->create_wall_timer(0.1s, std::bind(&ArCarController::receiveSerial, this));
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
  std::vector<uint8_t> array_data(data_.bin, data_.bin + 12);
  mutex_.unlock();
  std::vector<uint8_t> packet = EncodePacket(array_data);
  if (!serial_.send(packet)) {
    RCLCPP_ERROR(this->get_logger(), "Serial send error");
  }
};

void ArCarController::sendSerial2() {
  float accel = data_.accel * speed_limit_rate_;
  std::stringstream ss;
  ss << std::setprecision(2);
  ss << accel << "," << data_.steer << "," << data_.camera_direction;
  if (!serial_.send(ss.str())) {
    RCLCPP_ERROR(this->get_logger(), "Serial send error");
  }
};


void ArCarController::receiveSerial() {
  std::string receive = serial_.receive(true);
  std::cout << "receive = " << receive << std::endl;
};
