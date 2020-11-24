package main

import (
	"context"
	"encoding/json"
	"fmt"
	"log"
	"sync"

	mqtt "github.com/eclipse/paho.mqtt.golang"
	types "github.com/ssrc-tii/fog_sw/ros2_ws/src/communication_link/types"
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

func handleGPSMessages(ctx context.Context, mqttClient mqtt.Client) {
	messages := make (chan types.VehicleGlobalPosition)
	log.Printf("Creating subscriber for %s", "VehicleGlobalPosition")
	go Subscribe(messages)

	go func (){
		for m:=range messages{
			sendGPSLocation(mqttClient, Coordinates{m.Lat, m.Lon})
		}
	}()
}

func startTelemetry(ctx context.Context, wg *sync.WaitGroup, mqttClient mqtt.Client) {
	wg.Add(1)
	go func() {
		defer wg.Done()
		handleGPSMessages(ctx, mqttClient)
	}()
}
