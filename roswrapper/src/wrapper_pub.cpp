#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <stdio.h>

#include "wrapper_pub.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
  public:
    MinimalPublisher(char* topic,char* pub_name)
    : Node(pub_name), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>(topic, 10);
      RCLCPP_INFO(this->get_logger(), "topic: '%s' pub_name: '%s'", topic, pub_name);
    }
    void publish_callback(char* data)
    {
      auto message = std_msgs::msg::String();
      message.data = data; 
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
  void call_publish(void* obj, char* data)
  {
    MinimalPublisher* publisher = (MinimalPublisher*)obj;
    publisher->publish_callback(data);
  }

  int publish(void* callback, void* gopublisher, char* topic, char* name)
  {
    auto publisher = std::make_shared<MinimalPublisher>(topic,name);
    void (*go_callback)(void*,void*) = (void (*)(void*,void*))callback;
    go_callback((void*)&(*publisher),gopublisher);
    rclcpp::spin(publisher);
    return 0;
  }
#ifdef __cplusplus
}
#endif