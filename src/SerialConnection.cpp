#include "SerialConnection.hpp"
#include "rclcpp/rclcpp.hpp"

const int SerialConnection::kError = -1;

SerialConnection::SerialConnection() : port_(kError) {
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
  size_t send_size = str.size() + 1;
  char send_char[send_size];
  str.copy(send_char, str.size());
  send_char[str.size()] = '\0';
  int ret = write(port_, send_char, send_size);
  // std::cout << "ret = " << ret << ", size = " << send_size << ", data = " << str << std::endl;
  std::cout << "str = " << str << std::endl;
  return ret==static_cast<int>(send_size);
}

bool SerialConnection::send(const std::vector<uint8_t> &data) {
  size_t send_size = data.size();
  uint8_t array[send_size];
  std::copy(data.begin(), data.end(), array);

  int ret = write(port_, array, send_size);
  // std::cout << "ret = " << ret << ", size = " << send_size << std::endl;
  return ret==static_cast<int>(send_size);
}

std::string SerialConnection::receive(const bool wait, const char terminate) {
  std::string receive_str;
  bool receving = false;
  char receive_char;
  int count = 0;
  while (true) {
    int read_size = read(port_, &receive_char, 1);
    // std::cout << "read_size = " << read_size << ", receive_char = " << receive_char << std::endl;
    if (read_size > 0) {
      receving = true;
      receive_str.append(1, receive_char);
      if (receive_char == terminate) {
        break;
      }
    } else if (count > 20) {
      break; 
    } else {
      if (!wait || receving) {
        break;
      }
    }
    count++;
  }
  return receive_str;
}


