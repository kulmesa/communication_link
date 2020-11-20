#include <memory>

#include "wrapper_sub.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"
#include "px4_msgs/msg/vehicle_global_position.h"

using std::placeholders::_1;



class MinimalSubscriber : public rclcpp::Node
{
  public:

    MinimalSubscriber(void* callback)
    : Node("minimal_subscriber")
    {
      go_callback = (void (*)(int,void*))callback;
      subscription_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
          "VehicleGlobalPosition_PubSubTopic",
          10,
          [this](const px4_msgs::msg::VehicleGlobalPosition::ConstSharedPtr msg) {
            this->topic_callback(msg);
          });
    }

  private:
    void topic_callback(const px4_msgs::msg::VehicleGlobalPosition::ConstSharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "timestamp: '%lld' ", (long long)msg->timestamp);
      RCLCPP_INFO(this->get_logger(), "lat: '%f'", msg->lat);
      RCLCPP_INFO(this->get_logger(), "lon: '%f'", msg->lon);

//      const px4_msgs__msg__VehicleGlobalPosition* m = (const px4_msgs__msg__VehicleGlobalPosition*)msg; 
      px4_msgs__msg__VehicleGlobalPosition tmpmsg;
      tmpmsg.timestamp = msg->timestamp;
      tmpmsg.lon = msg->lon;
      tmpmsg.lat = msg->lat;
      go_callback(sizeof(tmpmsg),(void*)&tmpmsg);
      //go_callback(sizeof(px4_msgs__msg__VehicleGlobalPosition),(void*)&msg);
    }
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr subscription_;
    void (*go_callback)(int,void*);
};

#ifdef __cplusplus
extern "C" {
#endif
int subscribe(int argc, void* callback)
{
  char* argv = "sub";
//  void (*cb)() = (void (*)())callback;
//  rclcpp::init(argc, &argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>(callback));
//  rclcpp::shutdown();
  return 0;
}
#ifdef __cplusplus
}
#endif
