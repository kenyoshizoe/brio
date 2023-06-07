#include <thread>
#include <mutex>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

class BrioDriver : public rclcpp::Node {
 public:
  BrioDriver();
  void Poll();

 private:
  int fd_;
  std::mutex fd_mutex_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
  std::thread poll_thread_;
};
