#include "SerialConnection.hpp"
#include "rclcpp/rclcpp.hpp"

const int SerialConnection::kError = -1;

SerialConnection::SerialConnection(const bool enable_cobs = true) : port_(kError), enable_cobs_(enable_cobs) {
  // nop
}

SerialConnection::~SerialConnection() {
  if (port_ != kError) {
    tcsetattr(port_, TCSANOW, &old_settings_);
    close(port_);
  }
}

int SerialConnection::initialize(const std::string &device, const BaudRate &rate) {
  port_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (port_ == kError) {
    // RCLCPP_ERROR(this->get_logger(), "Device open error. device: %s", device.c_str());
    std::cout << "Device open error. device : \"" << device << "\""<< std::endl;
    return 1;
  }

  if (tcgetattr(port_, &old_settings_) == kError) {
    // RCLCPP_ERROR(this->get_logger(), "tcgetattr error.");
    std::cout << "tcgetattr error."<< std::endl;
    close(port_);
    return 2;
  }

  current_settings_ = old_settings_;

  if ((cfsetispeed(&current_settings_, rate) == kError) || (cfsetospeed(&current_settings_, rate) == kError)) {
    // RCLCPP_ERROR(this->get_logger(), "cfsetispeed or cfsetospeed error.");
    std::cout << "cfsetispeed or cfsetospeed error."<< std::endl;
    close(port_);
    return 3;
  }

  // current_settings_.c_iflag = IGNPAR;
  // current_settings_.c_oflag = 0;
  // current_settings_.c_lflag = 0;
  // current_settings_.c_cflag= (CS8 | CLOCAL | CREAD);

  current_settings_.c_cflag= (CS8 | CLOCAL | CREAD);
  current_settings_.c_cflag &= ~PARENB;
  current_settings_.c_cflag &= ~CSTOPB;
  current_settings_.c_cflag &= ~CSIZE;
  current_settings_.c_cflag |= CS8;
  current_settings_.c_cflag &= ~CRTSCTS;
  current_settings_.c_cflag |= CREAD | CLOCAL;
  current_settings_.c_iflag &= ~(IXON | IXOFF | IXANY);
  current_settings_.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  current_settings_.c_oflag &= ~OPOST;

  if (tcsetattr(port_, TCSANOW, &current_settings_) == kError) {
    std::cout << "tcsetattr error."<< std::endl;
    close(port_);
    return 4;
  }

  return 0;
}

bool SerialConnection::send(const std::string &str) {
  return send(str.c_str());
}

bool SerialConnection::send(const std::vector<uint8_t> &data) {
  size_t send_size = data.size();
  uint8_t array[send_size];
  std::copy(data.begin(), data.end(), array);

  uint8_t encoded_array[2 * send_size];
  if (enable_cobs_) {
    send_size = encode(array, send_size, encoded_array);
  }

  int ret = write(port_, encoded_array, send_size);
  // std::cout << "ret = " << ret << ", size = " << send_size << std::endl;
  return ret==static_cast<int>(send_size);
}

size_t SerialConnection::receive(const bool wait, const char* buffer, const size_t buffer_size, const char terminate = '\0') {
  bool receving = false;
  char receive_buffer[2 * buffer_size];
  char receive_char;
  int received_size = 0;
  while (true) {
    int read_size = read(port_, &receive_char, 1);
    if (read_size == 1) {
      receive_buffer[received_size] = receive_char;
      received_size++;
      receving = true;

      if (receive_char == terminate) {
        break;
      }
    } else if (received_size > buffer_size) {
      break; 
    } else {
      if (!wait || receving) {
        break;
      }
    }
  }

  if (enable_cobs_) {
    received_size = decode(receive_buffer, received_size, buffer);
  } else {
    std::memcpy(buffer, receive_buffer, received_size);
  }
  return received_size;
}

size_t SerialConnection::encode(const uint8_t* buffer, size_t size, uint8_t* encodedBuffer) {
  size_t read_index  = 0;
  size_t write_index = 1;
  size_t code_index  = 0;
  uint8_t code       = 1;

  while (read_index < size) {
    if (buffer[read_index] == 0) {
      encodedBuffer[code_index] = code;
      code = 1;
      code_index = write_index++;
      read_index++;
    } else {
      encodedBuffer[write_index++] = buffer[read_index++];
      code++;

      if (code == 0xFF) {
        encodedBuffer[code_index] = code;
        code = 1;
        code_index = write_index++;
      }
    }
  }
  encodedBuffer[code_index] = code;
  return write_index;
}

size_t SerialConnection::decode(const uint8_t* encodedBuffer, size_t size, uint8_t* decodedBuffer) {
  if (size == 0) return 0;

  size_t read_index  = 0;
  size_t write_index = 0;
  uint8_t code       = 0;
  uint8_t i          = 0;

  while (read_index < size) {
    code = encodedBuffer[read_index];

    if (read_index + code > size && code != 1) {
      return 0;
    }

    read_index++;

    for (i = 1; i < code; i++) {
      decodedBuffer[write_index++] = encodedBuffer[read_index++];
    }

    if (code != 0xFF && read_index != size) {
      decodedBuffer[write_index++] = '\0';
    }
  }
  return write_index;
}
