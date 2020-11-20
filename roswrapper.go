package main
import (
	"fmt"
	types "github.com/ssrc-tii/fog_sw/ros2_ws/src/communication_link/types"
)

/*
#cgo LDFLAGS: ${SRCDIR}/roswrapper/build/libgowrapper.so
#cgo LDFLAGS: -L${SRCDIR}/roswrapper/build
extern void GoCallback();
static inline void Callback(int size, void* data){
	GoCallback(size, data);
}
#include <roswrapper/include/wrapper_init.h>
static inline void init_rclcpp_c(){
	init_rclcpp();
}
static inline void shutdown_rclcpp_c(){
	shutdown_rclcpp();
}
#include <roswrapper/include/wrapper_pub.h>
static inline void publish_c(){
	char* argv = "gopublish";
	publish(1,&argv);
}
#include <roswrapper/include/wrapper_sub.h>
static inline void subscrice_c(){
	subscribe(1,&Callback);
}
*/
import "C"
import "unsafe"
//export GoCallback
func GoCallback(size C.int, data unsafe.Pointer){
	d := (*types.VehicleGlobalPosition)(data)
	fmt.Printf("callback size:%d time:%d\n", size , (*d).Timestamp) 
	fmt.Printf("callback size:%d lat:%f\n", size , (*d).Lat) 
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
	C.publish_c()
}

func Subscribe(){
	fmt.Println("subscribing")
	C.subscrice_c()
}




