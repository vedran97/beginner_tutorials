#include <chrono>
#include <functional>
#include <memory>
#include <rclcpp/executors.hpp>
#include <rclcpp/timer.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>

using std::placeholders::_1;

class StringPublisher : public rclcpp::Node {
 public:
  StringPublisher() : Node("string_publisher") {
    publisher_ =
        this->create_publisher<std_msgs::msg::String>("Problem_Pub", 10);
    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(1000),
                                std::bind(&StringPublisher::callback, this));
  }

 private:
  void callback() {
    auto message = std_msgs::msg::String();
    message.data = "Theres a problem houston";
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StringPublisher>());
  rclcpp::shutdown();
  return 0;
}