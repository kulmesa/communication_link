#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "wrapper_pub.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher(void* callback)
    : Node("minimal_publisher"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    }
    void publish_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = "Hello, world! " + std::to_string(count_++);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
 

  private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    size_t count_;
};

#ifdef __cplusplus
extern "C" {
#endif
  static MinimalPublisher* xyz;
  typedef void (MinimalPublisher::*PublishFn)();
  #define CALL_MEMBER_FN(object,ptrToMember)  ((object).*(ptrToMember))
  void call_publish(void* obj)
  {
    MinimalPublisher* publisher = (MinimalPublisher*)obj;
    publisher->publish_callback();
  }

  int publish(int argc, void* callback)
  {
    auto publisher = std::make_shared<MinimalPublisher>(callback);
    void (*go_callback)(void*) = (void (*)(void*))callback;
    go_callback((void*)&(*publisher));
    rclcpp::spin(publisher);
    return 0;
  }
#ifdef __cplusplus
}
#endif