package main

import (
	"context"
	"flag"
	"log"
	"os"
	"os/signal"
	"sync"
	"syscall"

	gstreamer "github.com/ssrc-tii/fog_sw/ros2_ws/src/communication_link/gstreamer"
	ros "github.com/ssrc-tii/fog_sw/ros2_ws/src/communication_link/ros"
	types "github.com/ssrc-tii/fog_sw/ros2_ws/src/communication_link/types"
)

import "C"

var (
	deviceID = flag.String("device_id", "", "The provisioned device id")
)

func main() {
	flag.Parse()
	// attach sigint & sigterm listeners
	terminationSignals := make(chan os.Signal, 1)
	signal.Notify(terminationSignals, syscall.SIGINT, syscall.SIGTERM)

	// quitFunc will be called when process is terminated
	ctx, quitFunc := context.WithCancel(context.Background())

	// wait group will make sure all goroutines have time to clean up
	var wg sync.WaitGroup

	ros.InitRosNode(*deviceID, "gstreamer_node")
	defer ros.ShutdownRosNode()
	startGstcmdListening(ctx, &wg)

	// wait for termination and close quit to signal all
	<-terminationSignals
	// cancel the main context
	log.Printf("Shuttding down..")
	quitFunc()

	// wait until goroutines have done their cleanup
	log.Printf("Waiting for routines to finish..")
	wg.Wait()
	log.Printf("Signing off - BYE")
}

func handleGstMessages(ctx context.Context) {
	messages := make(chan types.String)
	log.Printf("Creating subscriber for %s", "String")
	sub := ros.InitSubscriber(messages, "gstreamercmd", "std_msgs/msg/String")
	go sub.DoSubscribe(ctx)
	var ch chan bool
//	ch := make(chan bool)

	for m := range messages {
		msg := C.GoString((*C.char)(m.Data))
		log.Printf(msg)
		switch msg {
		case "start":
			ch = make(chan bool)
			go gstreamer.StartVideoStream(*deviceID, ch)
		case "stop":
			select {
			case <-ch:
			default:
				close(ch)
			}
		}
	}
	log.Printf("handleGstMessages END")
	sub.Finish()
}

func startGstcmdListening(ctx context.Context, wg *sync.WaitGroup) {
	wg.Add(1)
	go func() {
		defer wg.Done()
		handleGstMessages(ctx)
	}()
}
