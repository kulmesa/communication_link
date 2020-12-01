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
#cgo LDFLAGS: -L/opt/ros/foxy/lib -Wl,-rpath=/opt/ros/foxy/lib -lrcl -lrosidl_runtime_c -lrosidl_typesupport_cpp -lrosidl_typesupport_c -lstd_msgs__rosidl_generator_c -lstd_msgs__rosidl_typesupport_c -lrcutils 
#cgo CFLAGS: -I/opt/ros/foxy/include

#include "rcl/subscription.h"
#include "rcl/rcl.h"
#include "rcl/node.h"
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
//static inline int subscribe_(void* callback, char* topic, char* msgtype, char* name, int index,void * ts)
static inline int subscribe_( char* topic, char* msgtype, char* name, int index,void * ts)
{
  rcl_context_t * context_ptr;
  rcl_node_t * node_ptr;
  rcl_ret_t ret;
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  ret = rcl_init_options_init(&init_options, rcl_get_default_allocator());
  context_ptr = malloc(sizeof(rcl_context_t));
  *context_ptr = rcl_get_zero_initialized_context();
  ret = rcl_init(0, NULL, &init_options, context_ptr);

  node_ptr = malloc(sizeof(rcl_node_t));
  *node_ptr = rcl_get_zero_initialized_node();
  const char* name_ = "test_subscription_node";
  rcl_node_options_t node_options = rcl_node_get_default_options();
  ret = rcl_node_init(node_ptr, name_, "", context_ptr, &node_options);

  ////
//  const rosidl_message_type_support_t * ts =rosidl_typesupport_c__get_message_type_support_handle__px4_msgs__msg__VehicleGlobalPosition();
  //const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(px4_msgs, msg, VehicleGlobalPosition);

//  const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
  const char * topic_ = "/VehicleGlobalPosition_PubSubTopic";
  rcl_subscription_options_t subscription_options = rcl_subscription_get_default_options();
  rcutils_allocator_t allocator = rcutils_get_default_allocator();


  rcl_subscription_t my_sub = rcl_get_zero_initialized_subscription();
  ret = rcl_subscription_init (&my_sub, node_ptr, (const rosidl_message_type_support_t*)ts, topic_,&subscription_options);
  if (ret != RCL_RET_OK) {
    printf("Failed to create subscriber.\n");
    return -1;
  }


  rcl_serialized_message_t serialized_msg = rmw_get_zero_initialized_serialized_message();
  auto initial_capacity_ser = 0u;
  rmw_serialized_message_init(&serialized_msg, initial_capacity_ser, &allocator);
  for (;;){
    ret = rcl_take_serialized_message(&my_sub, &serialized_msg, NULL, NULL);
    usleep(100*1000);
    printf("rcl_take_serialized_message tesrt :%d\n", ret);
  }
  rcl_subscription_fini(&my_sub, node_ptr);
  rmw_serialized_message_fini(&serialized_msg);
  ret = rcl_node_fini(node_ptr);
  free( node_ptr);
  ret = rcl_shutdown(context_ptr);
  ret = rcl_context_fini(context_ptr);
  free( context_ptr);
 return 0;
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
	msgType := s.chanType.Elem()
	msg := reflect.New(msgType)
	method := msg.MethodByName("TypeSupport")
	result := method.Call(nil)

	C.subscribe_( C.CString(s.topic),  C.CString(s.msgtypestr),  C.CString(s.name), C.int(s.index), unsafe.Pointer(result[0].Pointer()))
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





