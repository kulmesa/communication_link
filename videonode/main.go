package main

import (
	"flag"
	"fmt"
	"log"
	"os"
	"os/exec"
	"os/signal"
	"syscall"
)

var (
	streamAddress = flag.String("stream-address", "", "Address to stream the video to")
)

func main() {
	flag.Parse()

	// attach sigint & sigterm listeners
	terminationSignals := make(chan os.Signal, 1)
	signal.Notify(terminationSignals, syscall.SIGINT, syscall.SIGTERM)

	gstArgs := []string{
		"udpsrc",
		"port=5600",
		"!",
		"application/x-rtp",
		"!",
		"rtph264depay",
		"!",
		"queue",
		"!",
		"rtspclientsink",
		"protocols=tcp",
		fmt.Sprintf("location=%s", *streamAddress),
	}

	gstCmd := exec.Command("gst-launch-1.0", gstArgs...)
	gstCmd.Stdout = os.Stdout
	gstCmd.Stderr = os.Stderr

	err := gstCmd.Start()
	if err != nil {
		log.Fatalf("Could not start gst-launch-1.0: %v", err)
	}

	go func() {
		<-terminationSignals

		gstCmd.Process.Kill()
	}()

	err = gstCmd.Wait()
	if err != nil {
		log.Fatalf("gst-launch-1.0 failed: %v", err)
	}

	log.Printf("Signing off - BYE")
}
