/**
 * @file publisher.cpp
 * @author Vedant Ranade
 * @brief ROS2 publisher node
 * @version 0.1
 * @date 2023-11-03
 *
 * @copyright Copyright (c) 2023
 *
 */
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

#include "beginner_tutorials/srv/detail/mod_string__struct.hpp"
#include "beginner_tutorials/srv/mod_string.hpp"
using std::placeholders::_1;
using std::placeholders::_2;

/**
 * @brief Class for the publisher node.Publishes a string to the topic
 * "Problem_Pub"
 */
class StringPublisher : public rclcpp::Node {
 public:
  StringPublisher() : Node("string_publisher") {
    publisher_ =
        this->create_publisher<std_msgs::msg::String>("Problem_Pub", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&StringPublisher::dataPublisherCallback, this));
    service_ = this->create_service<beginner_tutorials::srv::ModString>(
        "Problem_Srv",
        std::bind(&StringPublisher::serviceCallback, this, _1, _2));
  }

 private:
  void dataPublisherCallback() {
    auto message = std_msgs::msg::String();
    std::unique_lock<std::mutex> lock(dataMutex_);
    message.data = data;
    lock.unlock();
    RCLCPP_INFO_STREAM_ONCE(
        rclcpp::get_logger("rclcpp"),
        "Publishing: " << message.data << " on topic Problem_Pub");
    publisher_->publish(message);
  }
  void serviceCallback(
      const std::shared_ptr<beginner_tutorials::srv::ModString::Request>
          request,
      std::shared_ptr<beginner_tutorials::srv::ModString::Response> response) {
    RCLCPP_INFO_STREAM_ONCE(
        rclcpp::get_logger("rclcpp"),
        "received request with data " << request->publish_this);
    std::unique_lock<std::mutex> lock(dataMutex_);
    RCLCPP_WARN_STREAM_ONCE(
        rclcpp::get_logger("rclcpp"),
        "got the mutex,updating data to " << request->publish_this);
    data = request->publish_this;
    lock.unlock();
    RCLCPP_DEBUG_STREAM_ONCE(rclcpp::get_logger("rclcpp"), "unlocked mutex");
    response->set__success(true);
    RCLCPP_INFO_STREAM_ONCE(rclcpp::get_logger("rclcpp"),
                            "finished processing request");
  }
  std::mutex dataMutex_;
  std::string data = "Theres a problem houston";
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<beginner_tutorials::srv::ModString>::SharedPtr service_;
};
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StringPublisher>());
  rclcpp::shutdown();
  return 0;
}
