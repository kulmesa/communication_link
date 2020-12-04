package main
import (
	"fmt"
	types "github.com/ssrc-tii/fog_sw/ros2_ws/src/communication_link/types"
	"sync"
	"strings"
	"unsafe"
	"reflect"
	"time"
)

/*
#cgo LDFLAGS: -L/opt/ros/foxy/lib -L${SRCDIR}/../../install/px4_msgs/lib -Wl,-rpath=/opt/ros/foxy/lib -lrcl -lrosidl_runtime_c -lrosidl_typesupport_cpp -lrosidl_typesupport_c -lstd_msgs__rosidl_generator_c -lstd_msgs__rosidl_typesupport_c -lrcutils -lrmw_implementation -lpx4_msgs__rosidl_typesupport_c
#cgo CFLAGS: -I/opt/ros/foxy/include -I${SRCDIR}/../../install/px4_msgs/include/
#include "px4_msgs/msg/vehicle_global_position.h"
#include "rcutils/types/uint8_array.h"
#include "rcl/subscription.h"
#include "rcl/publisher.h"
#include "rcl/rcl.h"
#include "rcl/node.h"
#include "std_msgs/msg/string.h"

extern void GoCallback();
static inline void Callback(int size, void* data, void* name, int index){
	GoCallback(size, data, name, index);
}

typedef rcl_context_t* rcl_context_t_ptr;
typedef rcl_node_t* rcl_node_t_ptr;
typedef rcl_publisher_t* rcl_publisher_t_ptr;
typedef rcl_subscription_t* rcl_subscription_t_ptr;
typedef rcl_serialized_message_t* rcl_serialized_message_t_ptr;

typedef struct Publisher_C {
	 rcl_publisher_options_t pub_options;
	 rcl_publisher_t_ptr pub_ptr;
} publisher_t;

static inline void* init_ros_ctx(){
	rcl_ret_t ret;
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	ret = rcl_init_options_init(&init_options, rcl_get_default_allocator());
	if (ret != RCL_RET_OK) {
		printf("Failed to initialize.\n");
		return NULL;
	}
	rcl_context_t_ptr ctx_ptr = malloc(sizeof(rcl_context_t));
	*ctx_ptr = rcl_get_zero_initialized_context();
	ret = rcl_init(0, NULL, &init_options, ctx_ptr);
	if (ret != RCL_RET_OK) {
		printf("Failed to initialize rcl.\n");
		return NULL;
	}
	return (void*)ctx_ptr; 
}

static inline void* init_ros_node(void* ctx, char* name, char* namespace){
	rcl_ret_t ret;
	rcl_context_t_ptr ctx_ptr = (rcl_context_t_ptr)ctx;
	rcl_node_t_ptr node_ptr = malloc(sizeof(rcl_node_t));
	*node_ptr = rcl_get_zero_initialized_node();
	rcl_node_options_t node_options = rcl_node_get_default_options();
	ret = rcl_node_init(node_ptr, name, namespace, ctx_ptr, &node_options);
	if (ret != RCL_RET_OK) {
		printf("Failed to create node.\n");
		return NULL;
	}
	printf("Node created\n");
	return (void*)node_ptr; 
}

static inline void* init_publisher(void* ctx, void* node, char* topic){
	printf("init publisher begin\n"); 
	publisher_t* pub =malloc(sizeof(publisher_t));
	rcl_ret_t ret;
	rcl_context_t_ptr ctx_ptr = (rcl_context_t_ptr)ctx;
	rcl_node_t_ptr node_ptr = (rcl_node_t_ptr)node;	
	rcl_publisher_t publisher;
	rcl_publisher_options_t publisher_options;
	pub->pub_ptr = malloc(sizeof(rcl_publisher_t));
  	*pub->pub_ptr = rcl_get_zero_initialized_publisher();
  	pub->pub_options = rcl_publisher_get_default_options();
   	const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
	ret = rcl_publisher_init(pub->pub_ptr, node_ptr, ts, topic, &pub->pub_options);
	printf("init publisher after rcl_publisher_init\n"); 
  	return (void*)pub;
}

static inline void do_publish_c(void* publisher, char* data){
	rcl_publisher_t* pub = (rcl_publisher_t*)publisher;
	std_msgs__msg__String pub_msg;
	std_msgs__msg__String__init(&pub_msg);
	pub_msg.data.data = malloc(strlen(data)+1);
	strcpy(pub_msg.data.data,data);
	pub_msg.data.capacity = strlen(data)+1;
	pub_msg.data.size = strlen(data);
	rcl_ret_t ret = rcl_publish(pub, &pub_msg,NULL);
	if (ret != RCL_RET_OK)
	{
		printf("Failed to publish: %s\n", data);
	}
    std_msgs__msg__String__fini(&pub_msg);
}


typedef struct Subscriber_C {
	 rcl_subscription_t_ptr sub_ptr;
	 rcutils_allocator_t allocator;
	 rcl_serialized_message_t_ptr ser_msg_ptr;
} subscriber_t;

static inline void* init_subscriber(void* ctx, void* node, char* topic, char* msgtype,void* ts)
{
	subscriber_t* sub = malloc(sizeof(rcl_subscription_t));
	rcl_ret_t ret;
	rcl_context_t_ptr ctx_ptr = (rcl_context_t_ptr)ctx;
	rcl_node_t_ptr node_ptr = (rcl_node_t_ptr)node;	

	rcl_subscription_options_t subscription_options = rcl_subscription_get_default_options();
	sub->allocator = rcutils_get_default_allocator();

	sub->sub_ptr = malloc(sizeof(rcl_subscription_t));
	*sub->sub_ptr = rcl_get_zero_initialized_subscription();

	ret = rcl_subscription_init (sub->sub_ptr, node_ptr, (const rosidl_message_type_support_t*)ts, topic, &subscription_options);
	if (ret != RCL_RET_OK) {
		printf("Failed to create subscriber.\n");
		return NULL;
	}
	sub->ser_msg_ptr = malloc(sizeof(rcl_serialized_message_t));
	*sub->ser_msg_ptr = rmw_get_zero_initialized_serialized_message();
	int initial_capacity_ser = 0u;
	ret = rmw_serialized_message_init(sub->ser_msg_ptr, initial_capacity_ser, &sub->allocator);
	if (ret != RCL_RET_OK) {
		printf("Failed to create subscriber.\n");
		return NULL;
	}
	return (void*)sub;
}

static inline void* take_msg(void* sub, void* ser_msg, void* ts, int typesize,char* name, int index)
{
//	printf("take message called %d\n", typesize);
	rcl_subscription_t* s = (rcl_subscription_t*)sub;
	rcl_serialized_message_t* msg = (rcl_serialized_message_t*)ser_msg;
	rcl_ret_t ret = rcl_take_serialized_message(s, msg, NULL, NULL);
	if(ret == 0){
		uint8_t* deserialised_msg = (uint8_t*)malloc(typesize);
		ret = rmw_deserialize(msg, ts, deserialised_msg);
		Callback(0,(void*)(deserialised_msg), (void*)name, index);
		free (deserialised_msg);
	}
}
*/
import "C"

var global_messages chan <- types.VehicleGlobalPosition
var global_str_messages chan <- string
var wg sync.WaitGroup

var	ctx_ptr C.rcl_context_t_ptr;
var	node_ptr C.rcl_node_t_ptr;

func InitRosNode(namespace string){
	fmt.Println("init sros")
	ns := strings.ReplaceAll(namespace,"/","")
	ns = strings.ReplaceAll(ns,"-","")
	ctx_ptr = C.rcl_context_t_ptr(C.init_ros_ctx());
	node_ptr = C.rcl_node_t_ptr(C.init_ros_node(unsafe.Pointer(ctx_ptr), C.CString("communication_link_pub"),C.CString(ns)))
}

func ShutdownRosNode(){
	fmt.Println("shutdown ros")
	C.rcl_node_fini(node_ptr)
	C.free(unsafe.Pointer(node_ptr))
	C.rcl_shutdown(ctx_ptr)
	C.rcl_context_fini(ctx_ptr)
	C.free(unsafe.Pointer(ctx_ptr))
}

/////// Publisher ///////
type rclc_pub_ptrs_t struct{
	publisher_options C.rcl_publisher_options_t
	publisher_ptr C.rcl_publisher_t_ptr
}

type Publisher struct{
	rcl_ptrs *rclc_pub_ptrs_t
	msgtypestr string
	chanType reflect.Type
	chanValue reflect.Value
	publisher_ptr unsafe.Pointer
}

func InitPublisher(topic string) *Publisher{
	fmt.Println("init publisher:" + topic)
	pub := new(Publisher)
	pub.rcl_ptrs = (*rclc_pub_ptrs_t)(C.init_publisher(unsafe.Pointer(ctx_ptr),unsafe.Pointer(node_ptr), C.CString(topic)))
	return pub
}

func (p Publisher) DoPublish(data string){
	fmt.Println("do publish" , data)
	C.do_publish_c(unsafe.Pointer(p.rcl_ptrs.publisher_ptr),C.CString(data))
}

func (p Publisher) Finish(){
	//finish and clean rclc here
	C.rcl_publisher_fini(p.rcl_ptrs.publisher_ptr,node_ptr)
	C.free(unsafe.Pointer(p.rcl_ptrs.publisher_ptr))
}

/////// Subscriber ///////
type rclc_sub_ptrs_t struct{
	subscription_ptr C.rcl_subscription_t_ptr
	allocator C.rcutils_allocator_t
	ser_msg_ptr C.rcl_serialized_message_t_ptr
}

type Subscriber struct{
	name string
	topic string
	msgtypestr string
	chanType reflect.Type
	chanValue reflect.Value
	index int
	rcl_ptrs *rclc_sub_ptrs_t
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

	msg := reflect.New(msgType)
	method := msg.MethodByName("TypeSupport")
	result := method.Call(nil)	

	s.rcl_ptrs = (*rclc_sub_ptrs_t)(C.init_subscriber(
		unsafe.Pointer(ctx_ptr),
		unsafe.Pointer(node_ptr),
		C.CString(s.topic),
		C.CString(s.msgtypestr),
		unsafe.Pointer(result[0].Pointer()),
		))
	return s

}

func (s Subscriber)DoSubscribe(/*messages interface{},topic string, msgtype string*/){
	fmt.Println("subscribing")
	msgType := s.chanType.Elem()
	msg := reflect.New(msgType)
	method := msg.MethodByName("TypeSupport")
	result := method.Call(nil)	

	for{
		C.take_msg(unsafe.Pointer(s.rcl_ptrs.subscription_ptr),
			unsafe.Pointer(s.rcl_ptrs.ser_msg_ptr),
			unsafe.Pointer(result[0].Pointer()),
			C.int(msgType.Size()),
			C.CString(s.name),
			C.int(s.index) )
		time.Sleep(100*time.Millisecond)
	}
}

func (s Subscriber) Finish(){
	//finish and clean rclc here
	fmt.Println("Finish subscriber")
	C.rcutils_uint8_array_fini(s.rcl_ptrs.ser_msg_ptr);
	C.rcl_subscription_fini(s.rcl_ptrs.subscription_ptr,node_ptr)
	C.free(unsafe.Pointer(s.rcl_ptrs.subscription_ptr))
	fmt.Println("Finished subscriber")
}


//export GoCallback
func GoCallback(size C.int, data unsafe.Pointer, name unsafe.Pointer, index C.int){
	fmt.Println("Callback")
	sub := SubscriberArr[index]
	n := C.GoString((*C.char)(name))
	if (sub.name==n){
		msgType := sub.chanType.Elem()
		d := reflect.NewAt(msgType, data)
		sub.chanValue.Send(reflect.Indirect(d))
	}else{
		//if name does not match, search correct sub from array
		for _,s := range SubscriberArr{
			if s.name == n{
				msgType := s.chanType.Elem()
				d := reflect.NewAt(msgType, data)
				s.chanValue.Send(reflect.Indirect(d))
			}
		}
	}
}





