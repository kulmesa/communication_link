package main
import (
	"fmt"
	types "github.com/ssrc-tii/fog_sw/ros2_ws/src/communication_link/types"
)

/*
#cgo LDFLAGS: ${SRCDIR}/roswrapper/build/libgowrapper.so
#cgo LDFLAGS: -L${SRCDIR}/roswrapper/build
//#cgo CFLAGS: -I/opt/ros/foxy/include
//#cgo CFLAGS: -I/home/mika/fog/fog_docker/fog_sw/ros2_ws/install/px4_msgs/include 
//#cgo CFLAGS: -I/home/mika/fog/fog_docker/fog_sw/ros2_ws/install/px4_msgs/include/detail
//#include "px4_msgs/msg/vehicle_global_position.hpp"
extern void GoCallback();
static inline void Callback(int size, void* data){
	GoCallback(size, &data);
}
#include <roswrapper/include/wrapper_init.h>
static inline void init_rclcpp_c(){
	init_rclcpp();
}
static inline void shutdown_rclcpp_c(){
	shutdown_rclcpp();
}
#include <roswrapper/include/wrapper_pub.h>
static inline void pub(){
	char* argv = "pub";
	publish(1,&argv);
}
#include <roswrapper/include/wrapper_sub.h>
static inline void sub(){
//	char* argv = "sub";
	subscribe(1,&Callback);
}
*/
import "C"
import "unsafe"
//export GoCallback
func GoCallback(size C.int, data unsafe.Pointer){
	d := (*types.VehicleGlobalPosition)(data)
	fmt.Printf("sizeof VehicleGlobalPosition: %d\n", unsafe.Sizeof(d))
	fmt.Printf("callback size:%d data:%f\n", size , d.Lon) 
}

func InitRosContext(){
	fmt.Println("InitRosContext")
	C.init_rclcpp_c()
}

func ShutdownRosContext(){
	fmt.Println("ShutdownRosContext")
	C.shutdown_rclcpp_c()
}

func Publish(){
	fmt.Println("publishing")
	C.pub()
}

func Subscribe(){
	fmt.Println("subscribing")
	C.sub()
}




