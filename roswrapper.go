package main
// #cgo LDFLAGS: ${SRCDIR}/roswrapper/build/libgowrapper.so
// #cgo LDFLAGS: -L${SRCDIR}/roswrapper/build
// #include <roswrapper/include/wrapper_pub.h>
/* void pub(){
	char* argv = "pub";
	publish_(1,&argv);
}
*/
// #include <roswrapper/include/wrapper_sub.h>
/* void sub(){
	char* argv = "sub";
	subscribe_(1,&argv);
}
*/
import "C"
import "fmt"

func Publish(){
	fmt.Println("publishing")
	C.pub()
}

func Subscribe(){
	fmt.Println("subscribing")
	C.sub()
}




