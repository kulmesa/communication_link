package types

import "unsafe"
import "fmt"

/*
#cgo LDFLAGS: -L/opt/ros/foxy/lib -Wl,-rpath=/opt/ros/foxy/lib -lrcl -lrosidl_runtime_c -lrosidl_typesupport_cpp -lrosidl_typesupport_c -lstd_msgs__rosidl_generator_c -lstd_msgs__rosidl_typesupport_c
#cgo CFLAGS: -I/opt/ros/foxy/include
#include "std_msgs/msg/string.h"
#include "rosidl_runtime_c/string.h"
static inline const rosidl_message_type_support_t * ts_string(){
//    const rosidl_message_type_support_t * ts =rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__String();
       const rosidl_message_type_support_t * ts = ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String);
    return ts;
}
*/
import "C"

type String struct {
	Data *C.char
	Size int
    Capacity int
}

func (t* String) TypeSupport() unsafe.Pointer{
    fmt.Println("TypeSupport String called")
    return unsafe.Pointer(C.ts_string())
}

func (t* String) GetData() unsafe.Pointer{
    return unsafe.Pointer(t)
}

func (t* String) Finish() {
    //C.free(unsafe.Pointer(t.Data))
}

func GenerateString(data string) *String{
    t := new(String)
    t.Data = C.CString(data)
    t.Size = len(data)
    t.Capacity = t.Size+1
	return t
}

