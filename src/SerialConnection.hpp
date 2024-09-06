#pragma once

#include <fcntl.h>
#include <iostream>
#include <string>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include "3rdlib/PacketSerial/src/Encoding/COBS.h"

class SerialConnection {
 public:

  enum BaudRate {
    kB4800 = B4800,
    kB9600 = B9600,
    kB19200 = B19200,
    kB38400 = B38400,
    kB57600 = B57600,
    kB115200 = B115200,
   };

  SerialConnection(const bool enable_cobs);
  virtual ~SerialConnection();
  int initialize(const std::string &device, const BaudRate &baudrate);
  bool send(const std::string &str);
  bool send(const std::vector<uint8_t> &data);
  size_t receive(const bool wait, const char* buffer, const size_t buffer_size, const char terminate);

 private:
  size_t encode(const uint8_t* const buffer, size_t size, uint8_t* encodedBuffer) const;
  size_t decode(const uint8_t* const encodedBuffer, size_t size, uint8_t* decodedBuffer) const;

  static const int kError;
  termios old_settings_;
  termios current_settings_;
  int port_;
  bool enable_cobs_;
};


