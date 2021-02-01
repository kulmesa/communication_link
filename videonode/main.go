package main

import (
	"context"
	"encoding/json"
	"flag"
	"fmt"
	"log"
	"os"
	"os/signal"
	"sync"
	"syscall"

	gstreamer "github.com/tiiuae/communication_link/gst"
	ros "github.com/tiiuae/communication_link/ros"
	types "github.com/tiiuae/communication_link/types"
	conf "github.com/tiiuae/communication_link/videonode/config"
)

import "C"

var (
	deviceID                = flag.String("device_id", "", "The provisioned device id")
	configPath              = flag.String("config", "./config.yml", "The configuration of video feed")
	config     *conf.Config = nil
)

type gstreamerCmd struct {
	Command string
	Address string
	Source  string
}

type gstCh struct {
	Ch chan bool
	Source  string
}

func main() {
	flag.Parse()
	config = conf.NewConfig(*configPath)

	// attach sigint & sigterm listeners
	terminationSignals := make(chan os.Signal, 1)
	signal.Notify(terminationSignals, syscall.SIGINT, syscall.SIGTERM)

	// quitFunc will be called when process is terminated
	ctx, quitFunc := context.WithCancel(context.Background())

	// wait group will make sure all goroutines have time to clean up
	var wg sync.WaitGroup

	node := ros.InitRosNode(*deviceID, "videonode")
	defer node.ShutdownRosNode()
	startGstcmdListening(ctx, node, &wg)

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

func handleGstMessages(ctx context.Context, node *ros.Node) {
	messages := make(chan types.String)
	log.Printf("Creating subscriber for %s", "String")
	sub := node.InitSubscriber(messages, "videostreamcmd", "std_msgs/msg/String")
	go sub.DoSubscribe(ctx)
	chs := []*gstCh{}

	for m := range messages {
		msg := C.GoString((*C.char)(m.Data))
		log.Printf(msg)

		var gstCmd gstreamerCmd
		err := json.Unmarshal([]byte(msg), &gstCmd)
		if err != nil {
			log.Printf("Could not unmarshal gst command: %v", err)
			continue
		}
		switch gstCmd.Command {
		case "start":
			stopCh := new(gstCh)
			stopCh.Ch = make(chan bool)
			stopCh.Source = gstCmd.Source
			chs = append(chs, stopCh)
			go StartVideoStream(*deviceID, gstCmd.Address, gstCmd.Source, stopCh.Ch)
		case "stop":
			for i, stopCh := range chs{
				if gstCmd.Source == stopCh.Source{
					close(stopCh.Ch)
					chs = append(chs[:i],chs[i+1:]...)
				}
			}
		}
	}
	log.Printf("handleGstMessages END")
	sub.Finish()
}

func startGstcmdListening(ctx context.Context, node *ros.Node, wg *sync.WaitGroup) {
	wg.Add(1)
	go func() {
		defer wg.Done()
		handleGstMessages(ctx, node)
	}()
}

//StartVideoStream starts listening videostream and forward to rtsp server
func StartVideoStream(deviceID string, address string, source string, ch chan (bool)) {

	log.Println("StartVideoStream:", deviceID)

	pipelineStr := config.GetSource(source)
	pipelineStr += " name=source "
	pipelineStr += "! rtph264depay "
	rtspclientstr := fmt.Sprintf("! rtspclientsink name=sink protocols=tcp location=%s tls-validation-flags=generic-error",
		address)
	pipelineStr += rtspclientstr

	log.Println(pipelineStr)

	pipeline, err := gstreamer.New(pipelineStr)
	appsrc := pipeline.FindElement("source")

	appsrc.SetCap("application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264")

	if err != nil {
		log.Println("Pipeline failed")
		return
	}
	pipeline.Start()
	<-ch
	log.Println("End stream")
	pipeline.Stop()
}
