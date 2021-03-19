package missionengine

import (
	// "C"
	"context"
	"encoding/json"
	"fmt"
	"log"
	"sync"
	"time"

	mqtt "github.com/eclipse/paho.mqtt.golang"
	"github.com/tiiuae/communication_link/missionengine/gittransport"
	msg "github.com/tiiuae/communication_link/missionengine/types"
	"github.com/tiiuae/communication_link/missionengine/worldengine"
	"github.com/tiiuae/communication_link/ros"
	types "github.com/tiiuae/communication_link/types"
)

const (
	TOPIC_MISSION_ENGINE = "missionengine"
	TOPIC_MISSION_RESULT = "MissionResult_PubSubTopic"
)

type MissionEngine struct {
	ctx        context.Context
	wg         *sync.WaitGroup
	localNode  *ros.Node
	fleetNode  *ros.Node
	mqttClient mqtt.Client
	droneName  string
}

func New(ctx context.Context, wg *sync.WaitGroup, localNode *ros.Node, fleetNode *ros.Node, mqttClient mqtt.Client, droneName string) *MissionEngine {
	return &MissionEngine{ctx, wg, localNode, fleetNode, mqttClient, droneName}
}

func (me *MissionEngine) Start(gitServerAddress string, gitServerKey string) {
	ctx := me.ctx
	wg := me.wg
	droneName := me.droneName

	go func() {
		defer func() {
			r := recover()
			if r != nil {
				log.Printf("Recover: %v", r)
			}
		}()
		log.Printf("Starting mission engine of drone: '%s'", droneName)
		log.Printf("Running git clone...")
		gt := gittransport.New(gitServerAddress, gitServerKey)

		we := worldengine.New(droneName)

		messages := make(chan msg.Message)
		go runMessageLoop(ctx, wg, we, me.localNode, me.fleetNode, me.mqttClient, messages)
		go runGitTransport(ctx, wg, gt, messages, droneName)
		go runMissionEngineSubscriber(ctx, wg, messages, me.fleetNode, droneName)
		go runMissionResultSubscriber(ctx, wg, messages, me.localNode, droneName)
		// go runPublisher(ctx, wg, node)
	}()
}

func runMessageLoop(ctx context.Context, wg *sync.WaitGroup, we *worldengine.WorldEngine, localNode *ros.Node, fleetNode *ros.Node, mqttClient mqtt.Client, ch <-chan msg.Message) {
	wg.Add(1)
	defer wg.Done()

	pub := fleetNode.InitPublisher(TOPIC_MISSION_ENGINE, "std_msgs/msg/String", (*types.String)(nil))
	pubpath := localNode.InitPublisher("path", "nav_msgs/msg/Path", (*types.Path)(nil))
	pubmavlink := localNode.InitPublisher("mavlinkcmd", "std_msgs/msg/String", (*types.String)(nil))

	for m := range ch {
		log.Printf("Message received: %s: %s", m.MessageType, m.Message)
		messagesOut := we.HandleMessage(m, pubpath, pubmavlink)
		for _, r := range messagesOut {
			log.Printf("Message out: %v", r)
			if r.MessageType == "tasks-assigned" || r.MessageType == "task-completed" {
				b, err := json.Marshal(r)
				if err != nil {
					panic("Unable to marshal missionengine message")
				}
				pub.DoPublish(types.GenerateString(string(b)))
			}
			if r.MessageType == "mission-plan" {
				topic := fmt.Sprintf("/devices/%s/events/mission-plan", r.From)
				mqttClient.Publish(topic, 1, false, r.Message)
			}
			if r.MessageType == "flight-plan" {
				topic := fmt.Sprintf("/devices/%s/events/flight-plan", r.From)
				mqttClient.Publish(topic, 1, false, r.Message)
			}
		}
	}

	pub.Finish()
	pubpath.Finish()
	pubmavlink.Finish()
}

func runGitTransport(ctx context.Context, wg *sync.WaitGroup, gt *gittransport.GitEngine, ch chan<- msg.Message, droneName string) {
	wg.Add(1)
	defer wg.Done()

	for {
		select {
		case <-ctx.Done():
			close(ch)
			return
		case <-time.After(5 * time.Second):
		}
		msgs := gt.PullMessages(droneName)
		for _, msg := range msgs {
			ch <- msg
		}
	}
}

// func runPublisher(ctx context.Context, wg *sync.WaitGroup, node *ros.Node) {
// 	wg.Add(1)
// 	defer wg.Done()
// 	pub := node.InitPublisher(TOPIC_TASKS_ASSIGNED, "std_msgs/msg/String", (*types.String)(nil))

// 	msg := TasksAssigned{
// 		"droneb",
// 		[]string{"7bfa9df9-2b6f-4d20-84fa-905a6047190b", "e56982cd-b193-44a3-bcb8-d196f8f00209", "98b3b776-28af-4167-8a9f-12738c6a4c9b"},
// 	}

// 	for {
// 		select {
// 		case <-ctx.Done():
// 			pub.Finish()
// 			return
// 		case <-time.After(1000 * time.Millisecond):
// 		}
// 		str := types.GenerateString(serialize(msg))
// 		pub.DoPublish(str)
// 	}
// }

func runMissionEngineSubscriber(ctx context.Context, wg *sync.WaitGroup, ch chan<- msg.Message, node *ros.Node, droneName string) {
	wg.Add(1)
	defer wg.Done()
	rosMessages := make(chan types.String)
	sub := node.InitSubscriber(rosMessages, TOPIC_MISSION_ENGINE, "std_msgs/msg/String")
	go sub.DoSubscribe(ctx)

	for {
		select {
		case <-ctx.Done():
			sub.Finish()
			return
		case rosMsg := <-rosMessages:
			str := rosMsg.GetString()
			var m msg.Message
			err := json.Unmarshal([]byte(str), &m)
			if err != nil {
				panic("Unable to unmarshal missionengine message")
			}
			ch <- m
		}
	}
}

func runMissionResultSubscriber(ctx context.Context, wg *sync.WaitGroup, ch chan<- msg.Message, node *ros.Node, droneName string) {
	wg.Add(1)
	defer wg.Done()
	rosMessages := make(chan types.MissionResult)
	sub := node.InitSubscriber(rosMessages, TOPIC_MISSION_RESULT, "px4_msgs/msg/MissionResult")
	go sub.DoSubscribe(ctx)

	for {
		select {
		case <-ctx.Done():
			sub.Finish()
			return
		case rosMsg := <-rosMessages:
			b, err := json.Marshal(rosMsg)
			if err != nil {
				panic("Unable to marshal MissionResult")
			}
			ch <- msg.Message{
				Timestamp:   time.Now().UTC(),
				From:        droneName,
				To:          droneName,
				ID:          "",
				MessageType: "mission-result",
				Message:     string(b),
			}
		}
	}
}
