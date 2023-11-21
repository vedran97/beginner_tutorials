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
#include <stdexcept>
#include <string>

#include "beginner_tutorials/srv/mod_string.hpp"
using std::placeholders::_1;
using std::placeholders::_2;

/**
 * @brief Class for the publisher node.Publishes a string to the topic
 * "Problem_Pub"
 */
class StringPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new String Publisher object
   * @note starts a timer node and a service. Reads a parameter for time_period
   * and sets publisher timer to that value
   */
  StringPublisher() : Node("string_publisher") {
    this->declare_parameter("time_period_int_ms", static_cast<int>(1000));
    auto param = this->get_parameter("time_period_int_ms");
    if (param.get_type() == rclcpp::PARAMETER_NOT_SET) {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Time period Parameter not set");
      throw std::runtime_error("Time period Parameter not set");
    } else {
      RCLCPP_INFO_STREAM(this->get_logger(),
                         "Time period Parameter set to " << param.as_int());
    }
    int delayTime = 0;
    auto displayErrorSetTimeDelay = [&]() {
      RCLCPP_ERROR_STREAM(
          this->get_logger(),
          "Time period Parameter Too LOW, using spin rate as 1000ms");
      delayTime = 1000;
    };
    if (param.as_int() < 0) {
      RCLCPP_ERROR_STREAM(this->get_logger(),
                          "Time period Parameter IS NEGATIVE");
      displayErrorSetTimeDelay();
    } else if (param.as_int() < 1000) {
      displayErrorSetTimeDelay();
    } else {
      delayTime = param.as_int();
    }
    publisher_ =
        this->create_publisher<std_msgs::msg::String>("Problem_Pub", 10);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(delayTime),
        std::bind(&StringPublisher::dataPublisherCallback, this));
    service_ = this->create_service<beginner_tutorials::srv::ModString>(
        "Problem_Srv",
        std::bind(&StringPublisher::serviceCallback, this, _1, _2));
  }

 private:
  /**
   * @brief Publishes the string to the topic at a rate decided in the
   * constructor
   */
  void dataPublisherCallback() {
    auto message = std_msgs::msg::String();
    std::unique_lock<std::mutex> lock(dataMutex_);
    message.data = data;
    lock.unlock();
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing: "
                                               << "'" << message.data << "'"
                                               << " on topic 'Problem_Pub'");
    publisher_->publish(message);
  }
  /**
   * @brief Callback function for handling the service
   *
   * @param request contains the string to be published
   * @param response return success before exiting
   */
  void serviceCallback(
      const std::shared_ptr<beginner_tutorials::srv::ModString::Request>
          request,
      std::shared_ptr<beginner_tutorials::srv::ModString::Response> response) {
    RCLCPP_INFO_STREAM(this->get_logger(),
                       "received request with data " << request->publish_this);
    std::unique_lock<std::mutex> lock(dataMutex_);
    RCLCPP_WARN_STREAM(this->get_logger(), "got the mutex,updating data to "
                                               << request->publish_this);
    data = request->publish_this;
    lock.unlock();
    RCLCPP_DEBUG_STREAM(this->get_logger(), "unlocked mutex");
    response->set__success(true);
    RCLCPP_INFO_STREAM(this->get_logger(), "finished processing request");
  }
  std::mutex dataMutex_;
  std::string data = "Theres a problem houston";
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<beginner_tutorials::srv::ModString>::SharedPtr service_;
};
int main(int argc, char* argv[]) {
  // initialize the ros2 context
  rclcpp::init(argc, argv);
  // spin node
  rclcpp::spin(std::make_shared<StringPublisher>());
  // alert user about shutdown
  RCLCPP_FATAL_STREAM_ONCE(rclcpp::get_logger("rclpcpp"), "Publisher closed");
  // shutdown node
  rclcpp::shutdown();
  return 0;
}
