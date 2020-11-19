#include <memory>

#include "wrapper_sub.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"

using std::placeholders::_1;

extern "C" {

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {

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
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", "something");
    }
    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr subscription_;
};

int subscribe_(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
}