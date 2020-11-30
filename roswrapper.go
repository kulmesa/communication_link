package main
import (
	"fmt"
	types "github.com/ssrc-tii/fog_sw/ros2_ws/src/communication_link/types"
	"sync"
	"strings"
	"unsafe"
	"reflect"
)

/*
#cgo LDFLAGS: ${SRCDIR}/roswrapper/build/libgowrapper.so
#cgo LDFLAGS: -L${SRCDIR}/roswrapper/build
#cgo LDFLAGS: -L/opt/ros/foxy/lib -Wl,-rpath=/opt/ros/foxy/lib -lrcl -lrosidl_runtime_c -lrosidl_typesupport_cpp -lrosidl_typesupport_c -lstd_msgs__rosidl_generator_c -lstd_msgs__rosidl_typesupport_c
extern void GoCallback();
static inline void Callback(int size, void* data, void* name, int index){
	GoCallback(size, data, name, index);
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
static inline void subscrice_c(char* topic,char* msgtype, char* name, int index){
	subscribe(&Callback,topic,msgtype,name, index);
}
*/
import "C"

var global_messages chan <- types.VehicleGlobalPosition
var global_str_messages chan <- string
var wg sync.WaitGroup

func InitRosContext(){
	fmt.Println("InitRosContext")
	C.init_rclcpp_c()
}

func ShutdownRosContext(){
	fmt.Println("ShutdownRosContext")
	C.shutdown_rclcpp_c()
}

/////// Publisher ///////
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


/////// Subscriber ///////
type Subscriber struct{
	name string
	topic string
	msgtypestr string
	chanType reflect.Type
	chanValue reflect.Value
	index int
}
var SubscriberArr []Subscriber

func InitSubscriber(messages interface{},topic string, msgtype string) *Subscriber{
	fmt.Println("init subscriber")
	s := new(Subscriber)
	s.chanType = reflect.TypeOf(messages)
	s.chanValue = reflect.ValueOf(messages)
	msgType := s.chanType.Elem()
	fmt.Printf("%+v (%+v)\n", s.chanType, reflect.PtrTo(msgType))
	sub_name := "sub_" + strings.ReplaceAll(topic,"/","")
	s.name = sub_name
	s.topic = topic
	s.msgtypestr = msgtype
	SubscriberArr = append(SubscriberArr,*s)
	s.index = len(SubscriberArr)-1
	return s
}

//func Subscribe(messages chan <- types.VehicleGlobalPosition,topic string, msgtype string){
func (s Subscriber)DoSubscribe(/*messages interface{},topic string, msgtype string*/){
	fmt.Println("subscribing")
	C.subscrice_c( C.CString(s.topic),  C.CString(s.msgtypestr),  C.CString(s.name), C.int(s.index))
}

//export GoCallback
func GoCallback(size C.int, data unsafe.Pointer, name unsafe.Pointer, index C.int){
	fmt.Println("GoCallback size:%d , index:%d ", size, index)
	sub := SubscriberArr[index]
	n := C.GoString((*C.char)(name))
	if (sub.name==n){
		msgType := sub.chanType.Elem()
		fmt.Printf("%+v (%+v)\n", sub.chanType, msgType)
		d := reflect.NewAt(msgType, data)
		sub.chanValue.Send(reflect.Indirect(d))
	}else{
		//if name does not match, search correct sub from array
		for _,s := range SubscriberArr{
			if s.name == n{
				msgType := s.chanType.Elem()
				fmt.Printf("%+v (%+v)\n", s.chanType, msgType)
				d := reflect.NewAt(msgType, data)
				s.chanValue.Send(reflect.Indirect(d))
			}
		}
	}


}





