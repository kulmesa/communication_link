package main

import (
	"context"
	"encoding/json"
	"fmt"
	"log"
	"sync"
	uuid "github.com/google/uuid"
	mqtt "github.com/eclipse/paho.mqtt.golang"
	types "github.com/ssrc-tii/fog_sw/ros2_ws/src/communication_link/types"
)

const (
	qos    = 1
	retain = false
)

type telemetry struct {
	Coordinates coordinates
	DeviceID    string
}
type coordinates struct {
	Lat float64
	Lng float64
}
type rosCoordinates struct {
	Lat float64 `json:"lat"`
	Lon float64 `json:"lon"`
}

type sensorData struct {
	SensorData types.SensorCombined
	DeviceID    string
	MessageID		string
}

func sendGPSLocation(mqttClient mqtt.Client, coordinates coordinates) {
	topic := fmt.Sprintf("/devices/%s/%s", *deviceID, "events")
	t := telemetry{
		Coordinates: coordinates,
		DeviceID:    *deviceID,
	}
	b, _ := json.Marshal(t)
	mqttClient.Publish(topic, qos, retain, string(b))
}


func sendSensorData(mqttClient mqtt.Client, data types.SensorCombined) {
	topic := fmt.Sprintf("/devices/%s/%s", *deviceID, "events/sensordata")
	u := uuid.New()
	t := sensorData{
		SensorData: data,
		DeviceID:   *deviceID,
		MessageID: 		u.String(),
	}
	b, _ := json.Marshal(t)
	//log.Printf(string(b))
	mqttClient.Publish(topic, qos, retain, string(b))
}


func handleGPSMessages(ctx context.Context, mqttClient mqtt.Client) {
	messages := make (chan types.VehicleGlobalPosition)
	log.Printf("Creating subscriber for %s", "VehicleGlobalPosition")
	sub := initSubscriber(messages, "VehicleGlobalPosition_PubSubTopic","px4_msgs/msg/VehicleGlobalPosition")
	go sub.doSubscribe(ctx)
	go func (){
		for m:=range messages{
//			log.Printf("Lon: %f,  Lat:%f",m.Lat, m.Lon)
			sendGPSLocation(mqttClient, coordinates{m.Lat, m.Lon})
		}
	}()
	for {
		select {
		case <-ctx.Done():
			sub.finish()
			return
		}
	}
}

func handleSensorMessages(ctx context.Context, mqttClient mqtt.Client) {
	messages := make (chan types.SensorCombined)
	log.Printf("Creating subscriber for %s", "SensorCombined")
	sub := initSubscriber(messages, "SensorCombined_PubSubTopic","px4_msgs/msg/SensorCombined")
	go sub.doSubscribe(ctx)
	go func (){
		for m:=range messages{
			sendSensorData(mqttClient,m)
//			log.Printf("Timestamp: %v,  GyroRads:%v %v",m.Timestamp, m.GyroRad[0], m.GyroRad[1])
		}
	}()
	for {
		select {
		case <-ctx.Done():
			sub.finish()
			return
		}
	}
}


func startTelemetry(ctx context.Context, wg *sync.WaitGroup, mqttClient mqtt.Client) {
	wg.Add(2)
	go func() {
		defer wg.Done()
		handleGPSMessages(ctx, mqttClient)
	}()
	go func() {
		defer wg.Done()
		handleSensorMessages(ctx, mqttClient)
	}()
}
