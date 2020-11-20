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
      go_callback(sizeof(*msg),(void*)&(*msg));
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
  rclcpp::spin(std::make_shared<MinimalSubscriber>(callback));
  return 0;
}
#ifdef __cplusplus
}
#endif
