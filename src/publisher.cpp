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
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include <beginner_tutorials/srv/mod_string.hpp>
#include <chrono>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/string.hpp>
#include <stdexcept>
#include <string>
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
    tfTimer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000),
        std::bind(&StringPublisher::tf2PublisherTimerCallback, this));
    service_ = this->create_service<beginner_tutorials::srv::ModString>(
        "Problem_Srv",
        std::bind(&StringPublisher::serviceCallback, this, _1, _2));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  }

 private:
  void tf2PublisherTimerCallback() {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->get_clock()->now();
    t.header.frame_id = "world";
    t.child_frame_id = "talk";
    t.transform.translation.x = 1000;
    t.transform.translation.y = 1200;
    t.transform.translation.z = 0;
    tf2::Quaternion q;
    q.setRPY(M_PI, 0,0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();
    tf_broadcaster_->sendTransform(t);
  }
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
  rclcpp::TimerBase::SharedPtr tfTimer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Service<beginner_tutorials::srv::ModString>::SharedPtr service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
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
