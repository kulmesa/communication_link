package main
import (
	"fmt"
	types "github.com/ssrc-tii/fog_sw/ros2_ws/src/communication_link/types"
	"sync"
	"strings"
)

/*
#cgo LDFLAGS: ${SRCDIR}/roswrapper/build/libgowrapper.so
#cgo LDFLAGS: -L${SRCDIR}/roswrapper/build
#cgo LDFLAGS: -L/opt/ros/foxy/lib -Wl,-rpath=/opt/ros/foxy/lib -lrcl -lrosidl_runtime_c -lrosidl_typesupport_cpp -lrosidl_typesupport_c -lstd_msgs__rosidl_generator_c -lstd_msgs__rosidl_typesupport_c
extern void GoCallback();
static inline void Callback(int size, void* data){
	GoCallback(size, data);
}
extern void GoPublishCallback();
static inline void PublishCallback(void* cpublisher, void* gopublisher){
	GoPublishCallback(cpublisher,gopublisher);
}
#include <roswrapper/include/wrapper_init.h>
static inline void init_rclcpp_c(){
	init_rclcpp();
}
static inline void shutdown_rclcpp_c(){
	shutdown_rclcpp();
}
#include <roswrapper/include/wrapper_pub.h>
static inline void init_publisher(void* GoPublisher, char* topic, char* pub_name){
	publish(&PublishCallback,GoPublisher,topic, pub_name);
}
static inline void do_publish_c(void* publisher, char* data){
	call_publish(publisher, data);
}
#include <roswrapper/include/wrapper_sub.h>
static inline void subscrice_c(char* topic,char* msgtype, char* name){
	subscribe(&Callback,topic,msgtype,name);
}
*/
import "C"
import "unsafe"

var global_messages chan <- types.VehicleGlobalPosition
var wg sync.WaitGroup

//export GoCallback
func GoCallback(size C.int, data unsafe.Pointer){
	fmt.Println("GoCallback ", size)
	d := (*types.VehicleGlobalPosition)(data)
	global_messages <- *d
}

func InitRosContext(){
	fmt.Println("InitRosContext")
	C.init_rclcpp_c()
}

func ShutdownRosContext(){
	fmt.Println("ShutdownRosContext")
	C.shutdown_rclcpp_c()
}

type Publisher struct{
	publisher_ptr unsafe.Pointer 
}

func InitPublisher(topic string) *Publisher{
	fmt.Println("init publisher")
	pub := new(Publisher)
	pub_name := "pub_" + strings.ReplaceAll(topic,"/","")
	go C.init_publisher(unsafe.Pointer(pub), C.CString(topic), C.CString(pub_name))
	wg.Add(1)
	wg.Wait()
	return pub
}

func (p Publisher) DoPublish(data string){
	fmt.Println("do publish")
	C.do_publish_c(p.publisher_ptr,C.CString(data))
}

//export GoPublishCallback
func GoPublishCallback( publisher unsafe.Pointer, gopublisher unsafe.Pointer){
	gopub := (*Publisher)(gopublisher)
	gopub.publisher_ptr = publisher
	wg.Done()
}

func Subscribe(messages chan <- types.VehicleGlobalPosition,topic string, msgtype string){
	fmt.Println("subscribing")
	global_messages = messages
	sub_name := "sub_" + strings.ReplaceAll(topic,"/","")
	C.subscrice_c( C.CString(topic),  C.CString(msgtype),  C.CString(sub_name))
}




