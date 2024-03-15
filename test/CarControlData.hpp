#pragma once

#include <vector>

union CarControlData {
  struct {
    float steer;
    float accel;
    float camera_direction;
  };
  uint8_t bin[sizeof(float) * 3];
};
