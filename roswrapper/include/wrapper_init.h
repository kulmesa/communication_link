#ifndef GO_WRAPPER_INIT_H
#define GO_WRAPPER_INIT_H

#ifdef __cplusplus
extern "C" {
#endif
    void init_rclcpp();
    void shutdown_rclcpp();
#ifdef __cplusplus
}
#endif 

#endif // GO_WRAPPER_INIT_H