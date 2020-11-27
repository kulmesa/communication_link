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
	"github.com/ssrc-tii/rclgo"
	"github.com/ssrc-tii/rclgo/types"
)

type ControlCommand struct {
	Command   string
	Payload   string
	Timestamp time.Time
}

// handleControlCommand takes a command string and forwards it to mavlinkcmd
func handleControlCommand(command string, msg types.StdMsgsString, pub rclgo.Publisher) {
	var cmd ControlCommand
	err := json.Unmarshal([]byte(command), &cmd)
	if err != nil {
		log.Printf("Could not unmarshal command: %v", err)
		return
	}

	switch cmd.Command {
	case "takeoff":
		log.Printf("Publishing 'takeoff' to /mavlinkcmd")
		msg.SetText("takeoff")
		err := pub.Publish(msg.GetMessage(), msg.GetData())
		if err != nil {
			log.Fatalf("Publish failed: %v", err)
		}
	case "land":
		log.Printf("Publishing 'land' to /mavlinkcmd")
		msg.SetText("land")
		err := pub.Publish(msg.GetMessage(), msg.GetData())
		if err != nil {
			log.Fatalf("Publish failed: %v", err)
		}
	case "start_mission":
		log.Printf("Publishing 'start_mission' to /mavlinkcmd")
		msg.SetText("start_mission")
		err := pub.Publish(msg.GetMessage(), msg.GetData())
		if err != nil {
			log.Fatalf("Publish failed: %v", err)
		}
	case "pause_mission":
		log.Printf("Publishing 'pause_mission' to /mavlinkcmd")
		msg.SetText("pause_mission")
		err := pub.Publish(msg.GetMessage(), msg.GetData())
		if err != nil {
			log.Fatalf("Publish failed: %v", err)
		}
	case "resume_mission":
		log.Printf("Publishing 'resume_mission' to /mavlinkcmd")
		msg.SetText("resume_mission")
		err := pub.Publish(msg.GetMessage(), msg.GetData())
		if err != nil {
			log.Fatalf("Publish failed: %v", err)
		}
	case "return_home":
		log.Printf("Publishing 'return_home' to /mavlinkcmd")
		msg.SetText("return_home")
		err := pub.Publish(msg.GetMessage(), msg.GetData())
		if err != nil {
			log.Fatalf("Publish failed: %v", err)
		}
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

// handleControlCommands routine waits for commands and executes them. The routine quits when quit channel is closed
func handleControlCommands(ctx context.Context, wg *sync.WaitGroup, node rclgo.Node, commands <-chan string) {
	wg.Add(1)
	defer wg.Done()
	pub, closePub := newROSPublisher(node, "mavlinkcmd", types.GetMessageTypeFromStdMsgsString())
	defer closePub()

	var msg types.StdMsgsString
	msg.InitMessage()
	defer msg.DestroyMessage()

	for {
		select {
		case <-ctx.Done():
			return
		case command := <-commands:
			handleControlCommand(command, msg, pub)
		}
	}
}
func startCommandHandlers(ctx context.Context, wg *sync.WaitGroup, node rclgo.Node, mqttClient mqtt.Client) {

	controlCommands := make(chan string)
	//missionCommands := make(chan string)

	go handleControlCommands(ctx, wg, node, controlCommands)

	log.Printf("Subscribing to MQTT commands")
	commandTopic := fmt.Sprintf("/devices/%s/commands/", *DeviceID)
	token := mqttClient.Subscribe(fmt.Sprintf("%v#", commandTopic), 0, func(client mqtt.Client, msg mqtt.Message) {
		subfolder := strings.TrimPrefix(msg.Topic(), commandTopic)
		switch subfolder {
		case "control":
			log.Printf("Got control command: %v", string(msg.Payload()))
			controlCommands <- string(msg.Payload())
		case "mission":
			log.Printf("Got mission command, not yet supported")
		default:
			log.Printf("Unknown command subfolder: %v", subfolder)
		}
	})
	if err := token.Error(); err != nil {
		log.Fatalf("Error on subscribe: %v", err)
	}
}
