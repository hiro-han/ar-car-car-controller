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

  declare_parameter("device", "/dev/ttyACM1");

  control_info_subscription_ = this->create_subscription<ar_car_info::msg::ControlInfo>(
    "/unity/car_info",
    rclcpp::QoS(10),
    std::bind(&ArCarController::callback, this, std::placeholders::_1)
  );

  std::string device = get_parameter("device").as_string();
  if (!serial_.initialize(device, SerialConnection::kB9600)) {
    RCLCPP_ERROR(this->get_logger(), "Serial initialize error");
  }

  sleep(1);
  timer_ = this->create_wall_timer(0.1s, std::bind(&ArCarController::sendSerial2, this));
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
  std::stringstream ss;
  ss << std::setprecision(2);
  ss << data_.accel << "," << data_.steer << "," << data_.camera_direction;

  if (!serial_.send(ss.str())) {
    RCLCPP_ERROR(this->get_logger(), "Serial send error");
  }
};


void ArCarController::receiveSerial() {
  std::string receive = serial_.receive(true);
  std::cout << "receive = " << receive << std::endl;
};
