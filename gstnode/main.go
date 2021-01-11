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
)


var (
	deviceID          = flag.String("device_id", "", "The provisioned device id")
)

func main() {
	flag.Parse()
	// attach sigint & sigterm listeners
	terminationSignals := make(chan os.Signal, 1)
	signal.Notify(terminationSignals, syscall.SIGINT, syscall.SIGTERM)

	// quitFunc will be called when process is terminated
//	ctx, quitFunc := context.WithCancel(context.Background())
	_, quitFunc := context.WithCancel(context.Background())

	// wait group will make sure all goroutines have time to clean up
	var wg sync.WaitGroup

	ros.InitRosNode(*deviceID,"gstreamer_node")
	defer ros.ShutdownRosNode()
	go gstreamer.StartVideoStream()

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

