package types

import "unsafe"

/*
#cgo LDFLAGS: -L/opt/ros/foxy/lib -Wl,-rpath=/opt/ros/foxy/lib -lrcl -lrosidl_runtime_c -lrosidl_typesupport_cpp -lrosidl_typesupport_c -lstd_msgs__rosidl_generator_c -lnav_msgs__rosidl_typesupport_c -lnav_msgs__rosidl_generator_c
#cgo CFLAGS: -I/opt/ros/foxy/include
#include <nav_msgs/msg/path.h>
static inline const rosidl_message_type_support_t * ts_path(){
	const rosidl_message_type_support_t * ts =rosidl_typesupport_c__get_message_type_support_handle__nav_msgs__msg__Path();
    return ts;
}
*/
import "C"

//Path ROS Nav message struct
type Path struct {
	Header Header
	Poses  []PoseStamped
}

//TypeSupport ROS msg typesupport
func (t *Path) TypeSupport() unsafe.Pointer {
	return unsafe.Pointer(C.ts_path())
}

//GetData data getter
func (t *Path) GetData() unsafe.Pointer {
	return unsafe.Pointer(&(t.Poses[0]))
}

//Finish resource release
func (t *Path) Finish() {
	for i := 0; i < 4; i++ {
        C.free(unsafe.Pointer(t.Poses[i].Header.FrameID.Data))
    }		
}

//GeneratePath path generation
func GeneratePath(tmp int) *Path {

	//lat, lon, rel-alt, yaw at wp, time at wp (not used currently)
/*	(61.5022353,23.7748721,30.0,90,100e6),
	(61.5027688,23.7750600,30.0,110,200e6),
	(61.5004330,23.7752644,30.0,230,300e6),
	(61.5004168,23.7747874,30.0,90,400e6),
	(61.5025846,23.7736515,30.0,90,500e6),
	(61.5027580,23.7749010,30.0,90,600e6)]
*/

	t := new(Path)
	t.Poses = make([]PoseStamped, 4)

	for i := 0; i < 4; i++ {
		t.Poses[i].Header.FrameID.Data = C.CString("map")
		t.Poses[i].Header.FrameID.Size = 3
		t.Poses[i].Header.FrameID.Capacity = 4
		t.Poses[i].Header.Stamp = Time{100000, 1000000}
		t.Poses[i].Pose.Position.X = 47.39774149 + (float64)(tmp+i)*0.01
		t.Poses[i].Pose.Position.Y = 8.54559525 + (float64)(tmp+i)*0.01
		t.Poses[i].Pose.Position.Z = 500.0 + (float64)(tmp+i)
	
	}
/*	t.Poses[1].Header.FrameID.Data = C.CString("map")
	t.Poses[1].Header.FrameID.Size = 3
	t.Poses[1].Header.FrameID.Capacity = 4
	t.Poses[1].Header.Stamp = Time{100000, 1000000}
	t.Poses[1].Pose.Position.X = 1.0
	t.Poses[1].Pose.Position.Y = 2.0
	t.Poses[1].Pose.Position.Z = 3.0*/
	return t
}
