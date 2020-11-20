#include "wrapper_init.h"
#include "rclcpp/rclcpp.hpp"

#ifdef __cplusplus
extern "C" {
#endif
    void init_rclcpp()
     {
        char* argv = "goroswrapper";
        rclcpp::init(2, &argv);
     }
    void shutdown_rclcpp()
    {
        rclcpp::shutdown();
    }
#ifdef __cplusplus
}
#endif 