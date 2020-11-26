#include <memory>

#include "wrapper_sub.h"
#include "qos.hpp"
#include "rosbag2_node.hpp"
#include "generic_subscription.hpp"
#include "rclcpp/any_subscription_callback.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "px4_msgs/msg/vehicle_global_position.h"

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rcpputils/shared_library.hpp"
#include "rosidl_runtime_cpp/message_type_support_decl.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{
  public:

    MinimalSubscriber(void* callback)
    : Node("minimal_subscriber")
    {
      go_callback = (void (*)(int,void*))callback;
      subscription_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
//      subscription_ = this->create_subscription<rclcpp::AnySubscriptionCallback>(
          "VehicleGlobalPosition_PubSubTopic",
          10,
          [this](const px4_msgs::msg::VehicleGlobalPosition::ConstSharedPtr msg) {
//          [this](const void* msg) {
            this->topic_callback(msg);
          });
    }

  private:
    void topic_callback(const px4_msgs::msg::VehicleGlobalPosition::ConstSharedPtr msg) const
    {
      go_callback(sizeof(*msg),(void*)&(*msg));
    }
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr subscription_;
    void (*go_callback)(int,void*);
};

#ifdef __cplusplus
extern "C" {
#endif
namespace rosbag2_transport
{
int subscribe(int argc, void* callback)
{
  char* argv = "sub";
 // rclcpp::spin(std::make_shared<MinimalSubscriber>(callback));
//  return 0;
  auto node = std::make_shared<Rosbag2Node>("generic_subscriber");
  rclcpp::QoS qos = Rosbag2QoS::adapt_request_to_offers(
    "/VehicleGlobalPosition_PubSubTopic", node->get_publishers_info_by_topic("/VehicleGlobalPosition_PubSubTopic"));
  Rosbag2QoS subscription_qos{qos};
  void (*go_callback)(int,void*);
  go_callback = (void (*)(int,void*))callback;
  auto subscription = node->create_generic_subscription(
    "/VehicleGlobalPosition_PubSubTopic",
    "px4_msgs/msg/VehicleGlobalPosition",
    subscription_qos,
    [go_callback](std::shared_ptr<rclcpp::SerializedMessage> message) {
//      auto serializer = rclcpp::Serialization<MessageT>();
        // In order to deserialize the message we have to manually create a ROS2
        // message in which we want to convert the serialized data.
        using MessageT = px4_msgs::msg::VehicleGlobalPosition;
        MessageT string_msg;
        auto serializer = rclcpp::Serialization<MessageT>();
        serializer.deserialize_message(message.get(), &string_msg);
        go_callback(sizeof(string_msg),(void*)&(string_msg));
    });
  
  rclcpp::spin(node);
  return 0;
}
} //namespace rosbag2_transport

#ifdef __cplusplus
}
#endif
