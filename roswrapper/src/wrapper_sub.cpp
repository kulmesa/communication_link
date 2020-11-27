#include <memory>

#include "wrapper_sub.h"
#include "qos.hpp"
#include "rosbag2_node.hpp"
#include "generic_subscription.hpp"
#include "rclcpp/any_subscription_callback.hpp"
#include "rclcpp/rclcpp.hpp"
#include "typesupport_helpers.hpp"

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
    subscription_qos,
    [go_callback,msgtype](std::shared_ptr<rclcpp::SerializedMessage> message) {
        // In order to deserialize the message we have to manually create a ROS2
        // message in which we want to convert the serialized data.
        auto library = rosbag2_cpp::get_typesupport_library(msgtype, "rosidl_typesupport_cpp");
        auto typesupport = rosbag2_cpp::get_typesupport_handle(msgtype, "rosidl_typesupport_cpp", library);
        const int msgsize = (*message).size();
        uint8_t* deserialised_msg = (uint8_t*)malloc(msgsize);
        auto serializer = rclcpp::SerializationBase(typesupport);
        serializer.deserialize_message(message.get(), deserialised_msg);
        go_callback(msgsize,(void*)(deserialised_msg));
        free(deserialised_msg);
    });
  
  rclcpp::spin(node);
  return 0;
}
} //namespace rosbag2_transport

#ifdef __cplusplus
}
#endif
