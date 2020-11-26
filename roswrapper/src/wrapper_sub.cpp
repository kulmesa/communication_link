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

#ifdef __cplusplus
extern "C" {
#endif
namespace rosbag2_transport
{
int subscribe(void* callback, char* topic, char* msgtype, char* name)
{
  auto node = std::make_shared<Rosbag2Node>(name);
  rclcpp::QoS qos = Rosbag2QoS::adapt_request_to_offers(
    topic, node->get_publishers_info_by_topic(topic));
  Rosbag2QoS subscription_qos{qos};
  void (*go_callback)(int,void*);
  go_callback = (void (*)(int,void*))callback;
  auto subscription = node->create_generic_subscription(
    topic,
    msgtype,
//    "px4_msgs/msg/VehicleGlobalPosition",
    subscription_qos,
    [go_callback](std::shared_ptr<rclcpp::SerializedMessage> message) {
        // In order to deserialize the message we have to manually create a ROS2
        // message in which we want to convert the serialized data.
        using MessageT = px4_msgs::msg::VehicleGlobalPosition;
        MessageT deserialised_msg;
        auto serializer = rclcpp::Serialization<MessageT>();
        serializer.deserialize_message(message.get(), &deserialised_msg);
        go_callback(sizeof(deserialised_msg),(void*)&(deserialised_msg));
    });
  
  rclcpp::spin(node);
  return 0;
}
} //namespace rosbag2_transport

#ifdef __cplusplus
}
#endif
