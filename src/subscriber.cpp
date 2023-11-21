/**
 * @file subscriber.cpp
 * @author Vedant Ranade
 * @brief ROS2 publisher node
 * @version 0.1
 * @date 2023-11-03
 *
 * @copyright Copyright (c) 2023
 */

#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

/**
 * @brief Subscribes to Problem pub and prints received data to console
 */
class MinimalSubscriber : public rclcpp::Node {
 public:
  MinimalSubscriber() : Node("minimal_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "Problem_Pub", 10,
        std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

 private:
  /**
   * @brief Callback function for the subscriber. ROS LOGS received data
   *
   * @param msg
   */
  void topic_callback(const std_msgs::msg::String& msg) const {
    RCLCPP_INFO_STREAM(this->get_logger(), "I heard: "
                                               << "'" << msg.data << "'");
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[]) {
  // init ros2 cntext
  rclcpp::init(argc, argv);
  // spin node
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  // alert user about shutdown
  RCLCPP_FATAL_STREAM_ONCE(rclcpp::get_logger("rclpcpp"), "Subscriber closed");
  // shutdown ros2 context
  rclcpp::shutdown();
  return 0;
}
