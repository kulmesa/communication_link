package main

import (
	"context"
	"encoding/json"
	"fmt"
	"log"
	"strings"
	"sync"
	"time"

	mqtt "github.com/eclipse/paho.mqtt.golang"
	ros "github.com/ssrc-tii/fog_sw/ros2_ws/src/communication_link/ros"
	types "github.com/ssrc-tii/fog_sw/ros2_ws/src/communication_link/types"
)

type controlCommand struct {
	Command   string
	Payload   string
	Timestamp time.Time
}

// handleControlCommand takes a command string and forwards it to mavlinkcmd
func handleControlCommand(command string, pub *ros.Publisher) {
	var cmd controlCommand
	err := json.Unmarshal([]byte(command), &cmd)
	if err != nil {
		log.Printf("Could not unmarshal command: %v", err)
		return
	}

	switch cmd.Command {
	case "takeoff":
		log.Printf("Publishing 'takeoff' to /mavlinkcmd")
		pub.DoPublish(types.GenerateString("takeoff"))
	case "land":
		log.Printf("Publishing 'land' to /mavlinkcmd")
		pub.DoPublish(types.GenerateString("land"))
	case "start_mission":
		log.Printf("Publishing 'start_mission' to /mavlinkcmd")
		pub.DoPublish(types.GenerateString("start_mission"))
	case "pause_mission":
		log.Printf("Publishing 'pause_mission' to /mavlinkcmd")
		pub.DoPublish(types.GenerateString("pause_mission"))
	case "resume_mission":
		log.Printf("Publishing 'resume_mission' to /mavlinkcmd")
		pub.DoPublish(types.GenerateString("resume_mission"))
	case "return_home":
		log.Printf("Publishing 'return_home' to /mavlinkcmd")
		pub.DoPublish(types.GenerateString("return_home"))
	//case "plan":
	//	log.Printf("Publishing 'plan' to /mavlinkcmd")
	//	msg.SetText("plan")
	//	err := pub.Publish(msg.GetMessage(), msg.GetData())
	//	if err != nil {
	//		log.Fatalf("Publish failed: %v", err)
	//	}
	default:
		log.Printf("Unknown command: %v", command)
	}
}

// handleMissionCommand takes a command string and forwards it to mavlinkcmd
func handleMissionCommand(command string, pub *ros.Publisher) {
	var cmd controlCommand
	err := json.Unmarshal([]byte(command), &cmd)
	if err != nil {
		log.Printf("Could not unmarshal command: %v", err)
		return
	}
	switch cmd.Command {
	case "new_mission":
		log.Printf("Publishing mission to where ever")
		//		mission := new (types.PoseStamped)
		pub.DoPublish(types.GeneratePath())
	default:
		log.Printf("Unknown command: %v", command)
	}
}

// handleGstreamerCommand takes a command string and forwards it to gstreamercmd
func handleGstreamerCommand(command string, pub *ros.Publisher) {
	var cmd controlCommand
	err := json.Unmarshal([]byte(command), &cmd)
	if err != nil {
		log.Printf("Could not unmarshal command: %v", err)
		return
	}

	switch cmd.Command {
	case "start":
		log.Printf("Publishing 'start' to /gstreamercmd")
		pub.DoPublish(types.GenerateString("start"))
	default:
		log.Printf("Unknown command: %v", command)
	}
}

// handleControlCommands routine waits for commands and executes them. The routine quits when quit channel is closed
func handleControlCommands(ctx context.Context, wg *sync.WaitGroup, commands <-chan string) {
	wg.Add(1)
	defer wg.Done()
	pub := ros.InitPublisher("mavlinkcmd", "std_msgs/msg/String", (*types.String)(nil))
	for {
		select {
		case <-ctx.Done():
			pub.Finish()
			return
		case command := <-commands:
			handleControlCommand(command, pub)
		}
	}
}

// handleMissionCommands routine waits for commands and executes them. The routine quits when quit channel is closed
func handleMissionCommands(ctx context.Context, wg *sync.WaitGroup, commands <-chan string) {
	wg.Add(1)
	defer wg.Done()
	pub := ros.InitPublisher("whereever", "nav_msgs/msg/Path", (*types.Path)(nil))
	for {
		select {
		case <-ctx.Done():
			pub.Finish()
			return
		case command := <-commands:
			handleMissionCommand(command, pub)
		}
	}
}

// handleGstreamerCommands routine waits for commands and executes them. The routine quits when quit channel is closed
func handleGstreamerCommands(ctx context.Context, wg *sync.WaitGroup, commands <-chan string) {
	wg.Add(1)
	defer wg.Done()
	pub := ros.InitPublisher("gstreamercmd", "std_msgs/msg/String", (*types.String)(nil))

	for {
		select {
		case <-ctx.Done():
			pub.Finish()
			return
		case command := <-commands:
			handleGstreamerCommand(command, pub)
		}
	}
}

func startCommandHandlers(ctx context.Context, wg *sync.WaitGroup, mqttClient mqtt.Client) {

	controlCommands := make(chan string)
	missionCommands := make(chan string)
	gstreamerCommands := make(chan string)

	go handleControlCommands(ctx, wg, controlCommands)
	go handleMissionCommands(ctx, wg, missionCommands)
	go handleGstreamerCommands(ctx, wg, gstreamerCommands)

	log.Printf("Subscribing to MQTT commands")
	commandTopic := fmt.Sprintf("/devices/%s/commands/", *deviceID)
	token := mqttClient.Subscribe(fmt.Sprintf("%v#", commandTopic), 0, func(client mqtt.Client, msg mqtt.Message) {
		subfolder := strings.TrimPrefix(msg.Topic(), commandTopic)
		switch subfolder {
		case "control":
			log.Printf("Got control command: %v", string(msg.Payload()))
			controlCommands <- string(msg.Payload())
		case "mission":
			log.Printf("Got mission command")
			missionCommands <- string(msg.Payload())
		case "gstreamer":
			log.Printf("Got gstreamer command")
			gstreamerCommands <- string(msg.Payload())
		default:
			log.Printf("Unknown command subfolder: %v", subfolder)
		}
	})
	if err := token.Error(); err != nil {
		log.Fatalf("Error on subscribe: %v", err)
	}
}
