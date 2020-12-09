package types

import "unsafe"

/*
typedef char* _string_;
*/
import "C"


type String struct {
	Data unsafe.Pointer
//	Data string
	Size int
    Capacity int
}
