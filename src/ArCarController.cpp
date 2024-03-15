#include "ArCarController.hpp"
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "SerialPacket.hpp"
#include "SerialConnection.hpp"

using namespace std::chrono_literals;

ArCarController::ArCarController(const rclcpp::NodeOptions& options) : ArCarController("", options) {
  // nop
};

ArCarController::ArCarController(const std::string& name_space, const rclcpp::NodeOptions& options) :
  Node("ar_car_controller", name_space, options) {

  declare_parameter("device", "/dev/ttyACM0");

  control_info_subscription_ = this->create_subscription<ar_car_info::msg::ControlInfo>(
    "/unity/car_info",
    rclcpp::QoS(10),
    std::bind(&ArCarController::callback, this, std::placeholders::_1)
  );

  std::string device = get_parameter("device").as_string();
  if (!serial_.initialize(device, SerialConnection::kB115200)) {
    RCLCPP_ERROR(this->get_logger(), "Serial initialize error");
  }
}

void ArCarController::callback(const ar_car_info::msg::ControlInfo::SharedPtr msg){
  RCLCPP_INFO(this->get_logger(), "Receive: steer = %f, accel = %f, camera_direction = %f", msg->steer, msg->accel, msg->camera_direction);

  data_.accel = msg->accel;
  data_.steer = msg->steer;
  data_.camera_direction = msg->camera_direction;
  
  std::vector<uint8_t> array_data(data_.bin, data_.bin + 12);
  // https://qiita.com/hamukun8686/items/5c640420bde7ffe54d3c#%E5%88%A9%E7%94%A8%E4%BE%8B

  // https://www.delftstack.com/ja/howto/cpp/cpp-serial-communication/

  std::vector<uint8_t> packet = EncodePacket(array_data);
  if (!serial_.send(packet)) {
    RCLCPP_ERROR(this->get_logger(), "Serial send error");
  }
}


