package types

import "unsafe"

/*
#cgo LDFLAGS: -L/opt/ros/foxy/lib -Wl,-rpath=/opt/ros/foxy/lib -lrcl -lrosidl_runtime_c -lrosidl_typesupport_cpp -lrosidl_typesupport_c -lstd_msgs__rosidl_generator_c -lnav_msgs__rosidl_typesupport_c -lnav_msgs__rosidl_generator_c
#cgo CFLAGS: -I/opt/ros/foxy/include
#include "nav_msgs/msg/path.h"
static inline const rosidl_message_type_support_t * ts_path(){
	const rosidl_message_type_support_t * ts =rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__msg__Path();
    return ts;
}
*/
import "C"

//Path ROS Nav message struct
type Path struct {
    Header Header
    Poses []PoseStamped
}

//TypeSupport ROS msg typesupport
func (t* Path) TypeSupport() unsafe.Pointer{
    return unsafe.Pointer(C.ts_path())
}

//GetData data getter
func (t* Path) GetData() unsafe.Pointer{
    return unsafe.Pointer(&(t.Poses[0]))
}

//Finish resource release
func (t* Path) Finish() {
	C.free(unsafe.Pointer(t.Poses[0].Header.FrameID.Data))
	C.free(unsafe.Pointer(t.Poses[1].Header.FrameID.Data))
}

//GeneratePath path generation 
func GeneratePath() *Path{
    t := new(Path)
	t.Poses = make([]PoseStamped,2)
	
	t.Poses[0].Header.FrameID.Data = C.CString("map")
	t.Poses[0].Header.FrameID.Size = 3
	t.Poses[0].Header.FrameID.Capacity = 4
	t.Poses[0].Header.Stamp = Time{100000,1000000}
	t.Poses[0].Pose.Position.X = 1.0
	t.Poses[0].Pose.Position.Y = 2.0
	t.Poses[0].Pose.Position.Z = 3.0
	t.Poses[1].Header.FrameID.Data = C.CString("map")
	t.Poses[1].Header.FrameID.Size = 3
	t.Poses[1].Header.FrameID.Capacity = 4
	t.Poses[1].Header.Stamp = Time{100000,1000000}
	t.Poses[1].Pose.Position.X = 1.0
	t.Poses[1].Pose.Position.Y = 2.0
    t.Poses[1].Pose.Position.Z = 3.0
    return t
}

