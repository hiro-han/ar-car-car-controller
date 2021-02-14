#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Header.h"
#include "car_control_msgs/RcCarControl.h"
#include "controller_type.h"

ros::Publisher publisher;
ControllerType type = ControllerType::ProCon;
std::string controller_type;

void publishCarControl(const car_control_msgs::RcCarControl &msg) {
    publisher.publish(msg);
}

void joyCallback(const sensor_msgs::Joy& msg) {
  car_control_msgs::RcCarControl control;
  switch (type) {
  case ControllerType::ProCon:
    control.steer = msg.axes[0];
    control.speed = msg.axes[3];
    break;
  case ControllerType::PS3:
    control.steer = msg.axes[0];
    control.speed = msg.axes[4];
    break;
  case ControllerType::OculusQuest2:
    control.steer = -msg.axes[0];
    control.speed = msg.axes[3];
    break;
  default:
    control.steer = msg.axes[0];
    control.speed = msg.axes[3];
    break;
  }
  ROS_DEBUG("axis 0 = [%.3f], 4 = [%.3f]", control.steer, control.speed);
  publishCarControl(control);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "car_controller");
  ros::NodeHandle n;
  ros::NodeHandle pn("~");
  std::string type_str;
  std::string topic_name;

  pn.getParam("controller_type", type_str);
  if (type_str == "ProCon") {
    type = ControllerType::ProCon;
    topic_name = "joy";
  } else if (type_str == "PS3") {
    type = ControllerType::PS3;
    topic_name = "joy";
  } else if (type_str == "OculusQuest2") {
    type = ControllerType::OculusQuest2;
    topic_name = "oculus/controller";
  } else {
    type = ControllerType::OculusQuest2;
    topic_name = "oculus/controller";
  }

  ROS_DEBUG("Controller Type: %s", type_str.c_str());

  publisher = n.advertise<car_control_msgs::RcCarControl>("car_control", 100);

  ros::Subscriber subscriber = n.subscribe(topic_name, 10, joyCallback);
  ros::spin();

  return 0;
}