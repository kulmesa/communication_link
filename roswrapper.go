package main
import "fmt"

/*
#cgo LDFLAGS: ${SRCDIR}/roswrapper/build/libgowrapper.so
#cgo LDFLAGS: -L${SRCDIR}/roswrapper/build
extern void GoCallback();
static inline void Callback(){
	GoCallback();
}
#include <roswrapper/include/wrapper_pub.h>
static inline void pub(){
	char* argv = "pub";
	publish_(1,&argv);
}
#include <roswrapper/include/wrapper_sub.h>
static inline void sub(){
//	char* argv = "sub";
	subscribe_(1,&Callback);
}
*/
import "C"
//export GoCallback
func GoCallback(){
	fmt.Println("callback")
}


func Publish(){
	fmt.Println("publishing")
	C.pub()
}

func Subscribe(){
	fmt.Println("subscribing")
	C.sub()
}




