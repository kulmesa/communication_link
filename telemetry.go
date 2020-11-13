package main

import (
	"context"
	"encoding/json"
	"fmt"
	"log"
	"sync"
	"time"

	mqtt "github.com/eclipse/paho.mqtt.golang"
	"github.com/ssrc-tii/rclgo"
	"github.com/ssrc-tii/rclgo/types"
)

const (
	qos    = 1
	retain = false
)

type Telemetry struct {
	Coordinates Coordinates
	DeviceId    string
}
type Coordinates struct {
	Lat float64
	Lng float64
}
type ROSCoordinates struct {
	Lat float64 `json:"lat"`
	Lon float64 `json:"lon"`
}

func sendGPSLocation(mqttClient mqtt.Client, coordinates Coordinates) {
	topic := fmt.Sprintf("/devices/%s/%s", *DeviceID, "events")
	t := Telemetry{
		Coordinates: coordinates,
		DeviceId:    *DeviceID,
	}
	b, _ := json.Marshal(t)
	mqttClient.Publish(topic, qos, retain, string(b))
}

func handleGPSMessages(ctx context.Context, node rclgo.Node, mqttClient mqtt.Client) {
	sub, closeSub := newROSSubscription(node, "/VehicleGlobalPosition_temp", types.GetMessageTypeFromStdMsgsString())
	defer closeSub()

	//Creating the msg type
	var msg types.StdMsgsString
	msg.InitMessage()
	defer msg.DestroyMessage()

	for {
		// check for new messages
		err := sub.TakeMessage(&msg.MsgInfo, msg.GetData())
		if err == nil {
			// have a new message
			coordinates := msg.GetDataAsString()
			var rosCoord ROSCoordinates
			err := json.Unmarshal([]byte(coordinates), &rosCoord)
			if err != nil {
				log.Printf("Could not parse coordinates: %v", err)
			} else {
				sendGPSLocation(mqttClient, Coordinates{rosCoord.Lat, rosCoord.Lon})
			}
		}

		// check if time to quit
		// or sleep a bit
		select {
		case <-ctx.Done():
			return
		case <-time.After(100 * time.Millisecond):
			// continue to next message
		}
	}
}

func startTelemetry(ctx context.Context, wg *sync.WaitGroup, node rclgo.Node, mqttClient mqtt.Client) {
	wg.Add(1)
	go func() {
		defer wg.Done()

		handleGPSMessages(ctx, node, mqttClient)
	}()
}
