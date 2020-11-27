#include <memory>

#include "wrapper_sub.h"
#include "qos.hpp"
#include "rosbag2_node.hpp"
#include "generic_subscription.hpp"
#include "rclcpp/any_subscription_callback.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
//#include "px4_msgs/msg/vehicle_global_position.h"
#include "typesupport_helpers.hpp"
//#include "serialization_format_converter_factory.hpp"

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rcpputils/shared_library.hpp"
#include "rosidl_runtime_cpp/message_type_support_decl.hpp"
#include <fstream>
#include <iostream>
//#include "generic_message.hpp"

using std::placeholders::_1;


#ifdef __cplusplus
extern "C" {
#endif
namespace rosbag2_transport
{

typedef struct px4_msgs__msg__VehicleGlobalPosition
{
  uint64_t timestamp;
  double lat;
  double lon;
  float alt;
  float alt_ellipsoid;
  float delta_alt;
  uint8_t lat_lon_reset_counter;
  uint8_t alt_reset_counter;
  float eph;
  float epv;
  float terrain_alt;
  bool terrain_alt_valid;
  bool dead_reckoning;
} px4_msgs__msg__VehicleGlobalPosition__;

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
    [go_callback,msgtype](std::shared_ptr<rclcpp::SerializedMessage> message) {


 //       std::ofstream myfile;
 //       myfile.open ("testfile");
 //       myfile << (message.get()->get_rcl_serialized_message());
 //       myfile.close();
        // In order to deserialize the message we have to manually create a ROS2
        // message in which we want to convert the serialized data.
        auto library = rosbag2_cpp::get_typesupport_library(msgtype, "rosidl_typesupport_cpp");
        auto typesupport = rosbag2_cpp::get_typesupport_handle(msgtype, "rosidl_typesupport_cpp", library);

/*        using MessageT = px4_msgs::msg::VehicleGlobalPosition;
        MessageT deserialised_msg;
        auto serializer = rclcpp::Serialization<MessageT>();
        serializer.deserialize_message(message.get(), &deserialised_msg);
        go_callback(sizeof(deserialised_msg),(void*)&(deserialised_msg));
*/

          //char* deserialised_msg;
 //         px4_msgs__msg__VehicleGlobalPosition__ deserialised_msg;
          char deserialised_msg[56];
          auto serializer2 = rclcpp::SerializationBase(typesupport);
          serializer2.deserialize_message(message.get(), &deserialised_msg);

        
          go_callback(sizeof(deserialised_msg),(void*)(&deserialised_msg));

//        using MessageT = VehicleGlobalPositionTest;
//        
//        rosbag2_cpp::SerializationFormatConverterFactory factory;
//        std::unique_ptr<rosbag2_cpp::converter_interfaces::SerializationFormatDeserializer> cdr_deserializer_;
//        cdr_deserializer_ = factory.load_deserializer("cdr");
//        void *out_msg;
//        cdr_deserializer_->deserialize(*message.get(), typesupport, out_msg);
//        
//        go_callback(sizeof(*message.get()),(void*)&(*message.get()));
    });
  
  rclcpp::spin(node);
  return 0;
}
} //namespace rosbag2_transport

#ifdef __cplusplus
}
#endif
