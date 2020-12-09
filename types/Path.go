package types

import "unsafe"
import "fmt"

/*
#cgo LDFLAGS: -L/opt/ros/foxy/lib -Wl,-rpath=/opt/ros/foxy/lib -lrcl -lrosidl_runtime_c -lrosidl_typesupport_cpp -lrosidl_typesupport_c -lstd_msgs__rosidl_generator_c -lnav_msgs__rosidl_typesupport_c -lnav_msgs__rosidl_generator_c
#cgo CFLAGS: -I/opt/ros/foxy/include
#include "nav_msgs/msg/path.h"
static inline const rosidl_message_type_support_t * ts_(){
    const rosidl_message_type_support_t * ts =rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__msg__Path();
    return ts;
}
*/
import "C"
type Path struct {
    Header Header
    Poses []PoseStamped
}

func (t* Path) TypeSupport() unsafe.Pointer{
    fmt.Println("Path TypeSupport called")
    return unsafe.Pointer(C.ts_())
}
