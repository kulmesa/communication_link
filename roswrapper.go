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
extern void GoPublishCallback();
static inline void PublishCallback(void* cb, void* callit){
	GoPublishCallback(cb,callit);
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
	publish(1,&PublishCallback);
}
static inline void do_publish_c(void* publisher){
//	callback_struct* cb_ptr = (callback_struct*)cb;
	call_publish(publisher);
}
#include <roswrapper/include/wrapper_sub.h>
static inline void subscrice_c(){
	subscribe(1,&Callback);
}
*/
import "C"
import "unsafe"

var global_messages chan <- types.VehicleGlobalPosition
var publisher_ptr unsafe.Pointer

//export GoCallback
func GoCallback(size C.int, data unsafe.Pointer){
	d := (*types.VehicleGlobalPosition)(data)
	global_messages <- *d
	DoPublish()
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

func DoPublish(){
	fmt.Println("do publish")
	C.do_publish_c(publisher_ptr)
}

//export GoPublishCallback
func GoPublishCallback( publisher unsafe.Pointer){
	fmt.Println("publish callback called")
	publisher_ptr = publisher
}

func Subscribe(messages chan <- types.VehicleGlobalPosition){
	fmt.Println("subscribing")
	global_messages = messages
	C.subscrice_c()
}




