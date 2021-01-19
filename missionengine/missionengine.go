package missionengine

import (
	// "C"
	"context"
	"encoding/json"
	"log"
	"time"

	"github.com/ssrc-tii/fog_sw/ros2_ws/src/communication_link/ros"
	types "github.com/ssrc-tii/fog_sw/ros2_ws/src/communication_link/types"
)

import "C"

func Run(ctx context.Context) {
	go runSubscriber(ctx)
	go runPublisher(ctx)
}

type TasksAssigned struct {
	Name    string   `json:"name"`
	TaskIDs []string `json:"taskIds"`
}

type DroneState struct {
	Name      string
	TaskQueue []string
}

const (
	TOPIC_TASKS_ASSIGNED = "missionengine/tasks_assigned"
)

func runPublisher(ctx context.Context) {
	pub := ros.InitPublisher(TOPIC_TASKS_ASSIGNED, "std_msgs/msg/String", (*types.String)(nil))

	msg := TasksAssigned{
		"droneb",
		// []string{"7bfa9df9-2b6f-4d20-84fa-905a6047190b", "e56982cd-b193-44a3-bcb8-d196f8f00209", "98b3b776-28af-4167-8a9f-12738c6a4c9b"},
		[]string{},
	}
	_ = msg
	for i := 1; i < 20; i++ {
		time.Sleep(100 * time.Millisecond)
		s := "a"
		str := types.GenerateString(s)
		pub.DoPublish(str)
		// str := types.GenerateString("hello world")
		// pub.DoPublish(str)
	}
}

func runSubscriber(ctx context.Context) {
	messages := make(chan types.String)
	sub := ros.InitSubscriber(messages, TOPIC_TASKS_ASSIGNED, "std_msgs/msg/String")
	go sub.DoSubscribe(ctx)

	for m := range messages {
		// str := C.GoString((*C.char)(m.Data))
		// var msg TasksAssigned
		// deserialize(str, &msg)
		// log.Printf("Received: %v", msg)
		// log.Printf("Received: %v", str)
		log.Printf("Received: %v", m.Size)
	}
}

func serialize(i interface{}) string {
	b, err := json.Marshal(i)
	if err != nil {
		panic(err)
	}

	return string(b)
}

func deserialize(jsonString string, i interface{}) {
	err := json.Unmarshal([]byte(jsonString), i)
	if err != nil {
		panic(err)
	}
}
