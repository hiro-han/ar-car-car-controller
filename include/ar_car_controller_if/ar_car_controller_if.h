#include <rclcpp/rclcpp.hpp>
#include "ar_car_info/msg/control_info.h"
#include "serial_packet.h"

namespace ar_car_controller_if {

class ARCarControllerIF : public rclcpp::Node {

 public:
  MINIMAL_COMP_PUBLIC
  explicit ARCarControllerIF(const rclcpp::NodeOptions& options=rclcpp::NodeOptions());
  virtual ~ARCarControllerIF(){};

 private:
  int kSerialRetryNum = 10;

  void callbackControllerInfo();
  float calcAccel(const float input_key) const;
  void sendData();

  rclcpp::Publisher<ar_car_info::msg::EgoInfo>::SharedPtr publisher_;
  rclcpp::Subscription<ar_car_info::msg::ControlInfo>::SharedPtr subscription_;

  EgoInfoPacket ego_info_packet_;
};

}  // end of namespace ar_car_controller_if