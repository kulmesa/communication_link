package types

import "unsafe"

/*
#cgo LDFLAGS: -L/opt/ros/foxy/lib -Wl,-rpath=/opt/ros/foxy/lib -lrcl -lrosidl_runtime_c -lrosidl_typesupport_cpp -lrosidl_typesupport_c -lstd_msgs__rosidl_generator_c -lstd_msgs__rosidl_typesupport_c
#cgo CFLAGS: -I/opt/ros/foxy/include
#include "std_msgs/msg/string.h"
static inline const rosidl_message_type_support_t * ts_(){
    const rosidl_message_type_support_t * ts =rosidl_typesupport_c__get_message_type_support_handle__std_msgs__msg__String();
    return ts;
}
*/
import "C"

type String struct {
	Data unsafe.Pointer
	Size int
    Capacity int
}

func (t* String) TypeSupport() unsafe.Pointer{
    return unsafe.Pointer(C.ts_())
}

func (t* String) GetData() unsafe.Pointer{
    return unsafe.Pointer(t)
//    return t
}

func GenerateString(data string) *String{
	t := new(String)
	return t
}

