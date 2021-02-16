package telemetry

import (
	"context"
	"encoding/json"
	"fmt"
	"log"
	"sync"
	"time"

	mqtt "github.com/eclipse/paho.mqtt.golang"
	uuid "github.com/google/uuid"
	ros "github.com/tiiuae/communication_link/pkg/ros"
	types "github.com/tiiuae/communication_link/pkg/types"
)

const (
	qos    = 1
	retain = false
)

type telemetry struct {
	GpsData   types.VehicleGlobalPosition
	DeviceID  string
	MessageID string
}

type sensorData struct {
	SensorData types.SensorCombined
	DeviceID   string
	MessageID  string
}

var rosStartTime uint64 = 0

func sendGPSData(mqttClient mqtt.Client, gpsdata types.VehicleGlobalPosition) {
	topic := fmt.Sprintf("/devices/%s/%s", *deviceID, "events/location")
	u := uuid.New()
	if rosStartTime == 0 {
		rosStartTime = uint64(time.Now().UnixNano()/1000) - gpsdata.Timestamp
	}
	gpsdata.Timestamp = gpsdata.Timestamp + rosStartTime
	gpsdata.Timestamp_sample = gpsdata.Timestamp_sample + rosStartTime
	t := telemetry{
		GpsData:   gpsdata,
		DeviceID:  *deviceID,
		MessageID: u.String(),
	}
	b, _ := json.Marshal(t)
	mqttClient.Publish(topic, qos, retain, string(b))
}

func sendSensorData(mqttClient mqtt.Client, data types.SensorCombined) {
	topic := fmt.Sprintf("/devices/%s/%s", *deviceID, "events/sensordata")
	u := uuid.New()
	if rosStartTime == 0 {
		rosStartTime = uint64(time.Now().UnixNano()/1000) - data.Timestamp
	}
	data.Timestamp = data.Timestamp + rosStartTime
	t := sensorData{
		SensorData: data,
		DeviceID:   *deviceID,
		MessageID:  u.String(),
	}
	b, _ := json.Marshal(t)
	mqttClient.Publish(topic, qos, retain, string(b))
}

func handleGPSMessages(ctx context.Context, mqttClient mqtt.Client, node *ros.Node) {
	messages := make(chan types.VehicleGlobalPosition)
	log.Printf("Creating subscriber for %s", "VehicleGlobalPosition")
	sub := node.InitSubscriber(messages, "VehicleGlobalPosition_PubSubTopic", "px4_msgs/msg/VehicleGlobalPosition")
	go sub.DoSubscribe(ctx)
	for m := range messages {
		//			log.Printf("Lon: %f,  Lat:%f",m.Lat, m.Lon)
		sendGPSData(mqttClient, m)
	}
	log.Printf("handleGPSMessages END")
	sub.Finish()
}

func handleSensorMessages(ctx context.Context, mqttClient mqtt.Client, node *ros.Node) {
	messages := make(chan types.SensorCombined)
	log.Printf("Creating subscriber for %s", "SensorCombined")
	sub := node.InitSubscriber(messages, "SensorCombined_PubSubTopic", "px4_msgs/msg/SensorCombined")
	go sub.DoSubscribe(ctx)
	for m := range messages {
		sendSensorData(mqttClient, m)
		//			log.Printf("Timestamp: %v,  GyroRads:%v %v",m.Timestamp, m.GyroRad[0], m.GyroRad[1])
	}
	log.Printf("handleSensorMessages END")
	sub.Finish()
}

func startTelemetry(ctx context.Context, wg *sync.WaitGroup, mqttClient mqtt.Client, node *ros.Node) {
	wg.Add(2)
	go func() {
		defer wg.Done()
		handleGPSMessages(ctx, mqttClient, node)
	}()
	go func() {
		defer wg.Done()
		handleSensorMessages(ctx, mqttClient, node)
	}()
}