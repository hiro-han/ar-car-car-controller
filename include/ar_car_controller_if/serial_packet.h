
class ControlInfoPacket {
  float steer;
  float accel;
  float camera_direction;

  ControlInfoPacket() {
    steer = 0.0f;
    accel = 0.0f;
    camera_direction = 0.0f;
  }

  virtual ~ControlInfoPacket() {}

  // void writeToPacket(unsigned char &packet) {
  //   unsigned int position = packet;
  //   size_t size = sizeof(steer);
    
  //   std::memcpy(&steer, position, size);
  //   position += size;

  //   size = sizeof(accel);
  //   std::memcpy(&accel, position, size);
  //   position += size;

  //   size = sizeof(camera_direction);
  //   std::memcpy(&camera_direction, position, size);
  //   position += size;
  // }
};

class EgoInfoPacket {
  unsigned int counter;
  float steer;
  float accel;
  float camera_direction;

  EgoInfoPacket() {
    counter = 0;
    steer = 0.0f;
    accel = 0.0f;
    camera_direction = 0.0f;
  }

  virtual ~ControlInfoPacket() {}

  // void readFromPacket(unsigned char &packet) {

  // }
};