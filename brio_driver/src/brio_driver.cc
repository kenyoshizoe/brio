#include "brio_driver/brio_driver.h"

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <istream>
#include <queue>
#include <string>

#include "brio_driver/utils/cobs.h"
#include "main.pb.h"

BrioDriver::BrioDriver() : Node("brio_driver") {
  // Declare parameters.
  this->declare_parameter("device_name", "/dev/ttyUSB0");
  // Get parameters for serial port.
  std::string device_name =
      this->get_parameter("device_name").as_string();  // Open serial port.
  fd_ = open(device_name.c_str(), O_RDWR);
  if (fd_ < 0) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
    return;
  }
  // Configure serial port.
  struct termios tio;
  tcgetattr(fd_, &tio);
  cfsetispeed(&tio, B115200);
  cfsetospeed(&tio, B115200);
  cfmakeraw(&tio);
  tio.c_iflag = 0;
  tio.c_oflag = 0;
  tio.c_cflag |= CREAD;    // 受信有効
  tio.c_cflag |= CLOCAL;   // ローカルライン（モデム制御なし）
  tio.c_cflag |= CS8;      // データビット:8bit
  tio.c_lflag &= ~ICANON;  // 非カノニカルモード
  tio.c_lflag &= ~ECHO;    // エコー無効
  tio.c_cc[VMIN] = 0;
  tio.c_cc[VTIME] = 0;
  tcsetattr(fd_, TCSANOW, &tio);

  RCLCPP_INFO(this->get_logger(), "Opened serial port.");

  joint_states_pub_ =
      this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  poll_thread_ = std::thread([&]() { this->Poll(); });
}

void BrioDriver::Poll() {
  std::queue<uint8_t> rx_queue_;
  while (true) {
    std::this_thread::yield();
    uint8_t buf[256];
    size_t size = 0;
    {
      std::unique_lock<std::mutex> lock(fd_mutex_);
      size = ::read(fd_, buf, 256);
    }
    for (size_t i = 0; i < size; i++) {
      rx_queue_.push(buf[i]);
      if (rx_queue_.back() != 0x00) {
        continue;
      }
      // COBS decode.
      std::vector<uint8_t> rx_buf;
      while (!rx_queue_.empty()) {
        uint8_t c = rx_queue_.front();
        rx_queue_.pop();
        rx_buf.push_back(c);
      }
      rx_buf = brio::COBS::Decode(rx_buf);
      if (rx_buf.size() < 2) {
        std::queue<uint8_t> empty;
        std::swap(rx_queue_, empty);
        continue;
      }
      // Check checksum.
      uint8_t checksum = 0;
      for (size_t i = 0; i < rx_buf.size() - 1; i++) {
        checksum += rx_buf[i];
      }
      if (checksum != rx_buf.back()) {
        std::queue<uint8_t> empty;
        std::swap(rx_queue_, empty);
        continue;
      }
      rx_buf.pop_back();
      // Parse message.
      std::string rx_buf_str(rx_buf.begin(), rx_buf.end());
      brio::PC2Robot msg;
      msg.ParseFromString(rx_buf_str);
      // Clear rx_queue_.
      std::queue<uint8_t> empty;
      std::swap(rx_queue_, empty);
      // Publish joint states.
      sensor_msgs::msg::JointState joint_states_msg;
      joint_states_msg.header.stamp = this->now();
      joint_states_msg.name = {"link_2_joint", "link_3_joint", "link_4_joint"};
      joint_states_msg.position = {msg.j1().position(), msg.j2().position(),
                                   msg.j3().position()};
      joint_states_msg.velocity = {msg.j1().velocity(), msg.j2().velocity(),
                                   msg.j3().velocity()};
      joint_states_pub_->publish(joint_states_msg);
    }
  }
}
