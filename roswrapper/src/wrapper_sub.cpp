#include <memory>

#include "wrapper_sub.h"
#include "qos.hpp"
#include "rosbag2_node.hpp"
#include "generic_subscription.hpp"
#include "rclcpp/any_subscription_callback.hpp"
#include "rclcpp/rclcpp.hpp"
#include "typesupport_helpers.hpp"

#include "rcl/subscription.h"
#include "rcl/rcl.h"
#include "rcl/node.h"
#include "std_msgs/msg/string.h"
#include "px4_msgs/msg/vehicle_global_position.h"

using std::placeholders::_1;


#ifdef __cplusplus
extern "C" {
#endif
namespace rosbag2_transport
{

int subscribe(void* callback, char* topic, char* msgtype, char* name, int index)
{
  rcl_context_t * context_ptr;
  rcl_node_t * node_ptr;
  rcl_ret_t ret;
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  ret = rcl_init_options_init(&init_options, rcl_get_default_allocator());
  context_ptr = new rcl_context_t;
  *context_ptr = rcl_get_zero_initialized_context();
  ret = rcl_init(0, nullptr, &init_options, context_ptr);

  node_ptr = new rcl_node_t;
  *node_ptr = rcl_get_zero_initialized_node();
  constexpr char name_[] = "test_subscription_node";
  rcl_node_options_t node_options = rcl_node_get_default_options();
  ret = rcl_node_init(node_ptr, name_, "", context_ptr, &node_options);


  ////
  const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(px4_msgs, msg, VehicleGlobalPosition);
//  const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
  const char * topic_ = "/VehicleGlobalPosition_PubSubTopic";
  rcl_subscription_options_t subscription_options;
  rcl_subscription_t subscription;
  rcl_subscription_t subscription_zero_init;
  rcutils_allocator_t allocator;

  ///





  ret = rcl_node_fini(node_ptr);
  delete node_ptr;
  ret = rcl_shutdown(context_ptr);
  ret = rcl_context_fini(context_ptr);
  delete context_ptr;

  /*
  auto node = std::make_shared<Rosbag2Node>(name);
  rclcpp::QoS qos = Rosbag2QoS::adapt_request_to_offers(
    topic, node->get_publishers_info_by_topic(topic));
  Rosbag2QoS subscription_qos{qos};
  void (*go_callback)(int,void*, void*, int);
  go_callback = (void (*)(int,void*, void*, int))callback;
  auto library = rosbag2_cpp::get_typesupport_library(msgtype, "rosidl_typesupport_cpp");
  auto typesupport = rosbag2_cpp::get_typesupport_handle(msgtype, "rosidl_typesupport_cpp", library);

  auto subscription = node->create_generic_subscription(
    topic,
    msgtype,
    subscription_qos,
    [go_callback,msgtype,typesupport, name, index](std::shared_ptr<rclcpp::SerializedMessage> message) {
        // In order to deserialize the message we have to manually create a ROS2
        // message in which we want to convert the serialized data.
        const int msgsize = (*message).size();
        char* deserialised_msg = (char*)malloc(msgsize);
        auto serializer = rclcpp::SerializationBase(typesupport);
        serializer.deserialize_message(message.get(), deserialised_msg);
        go_callback(msgsize,(void*)(deserialised_msg), (void*)name, index);
        free(deserialised_msg);
    });
  
  rclcpp::spin(node); */
  return 0;
}
} //namespace rosbag2_transport

#ifdef __cplusplus
}
#endif
