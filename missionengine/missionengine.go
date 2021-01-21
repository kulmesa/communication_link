package missionengine

import (
	// "C"
	"context"
	"log"
	"sync"
	"time"

	"github.com/tiiuae/communication_link/missionengine/gittransport"
	msg "github.com/tiiuae/communication_link/missionengine/types"
	"github.com/tiiuae/communication_link/missionengine/worldengine"
	"github.com/tiiuae/communication_link/ros"
	types "github.com/tiiuae/communication_link/types"
)

import "C"

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

type MissionEngine struct {
	ctx       context.Context
	wg        *sync.WaitGroup
	node      *ros.Node
	droneName string
}

func New(ctx context.Context, wg *sync.WaitGroup, node *ros.Node, droneName string) *MissionEngine {
	return &MissionEngine{ctx, wg, node, droneName}
}

func (me *MissionEngine) Start(gitServerAddress string, gitServerKey string) {
	ctx := me.ctx
	wg := me.wg
	node := me.node
	droneName := me.droneName

	go func() {
		log.Printf("Starting mission engine of drone: '%s'", droneName)
		log.Printf("Running git clone...")
		gt := gittransport.New()
		drones := gt.Config.GetFleetNames()
		log.Printf("Fleet discovered: %v", drones)

		leader := drones[0]
		if droneName == leader {
			log.Printf("Elected as leader")
		}

		we := worldengine.New(droneName, drones, leader)

		messages := make(chan msg.Message)
		go runMessageLoop(ctx, wg, we, node, messages)
		go runGitTransport(ctx, wg, gt, messages, droneName)
		go runSubscriber(ctx, wg, messages, node, droneName)
		// go runPublisher(ctx, wg, node)
	}()

}

func runMessageLoop(ctx context.Context, wg *sync.WaitGroup, we *worldengine.WorldEngine, node *ros.Node, ch <-chan msg.Message) {
	wg.Add(1)
	defer wg.Done()

	pub := node.InitPublisher(TOPIC_TASKS_ASSIGNED, "std_msgs/msg/String", (*types.String)(nil))

	for m := range ch {
		log.Printf("Message received: %s: %s", m.MessageType, m.Message)
		messagesOut := we.HandleMessage(m)
		for _, r := range messagesOut {
			log.Printf("Message out: %v", r)
			pub.DoPublish(types.GenerateString(r.Message))
		}
	}

	pub.Finish()
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

func runSubscriber(ctx context.Context, wg *sync.WaitGroup, ch chan<- msg.Message, node *ros.Node, droneName string) {
	wg.Add(1)
	defer wg.Done()
	rosMessages := make(chan types.String)
	sub := node.InitSubscriber(rosMessages, TOPIC_TASKS_ASSIGNED, "std_msgs/msg/String")
	go sub.DoSubscribe(ctx)

	for {
		select {
		case <-ctx.Done():
			sub.Finish()
			return
		case rosMsg := <-rosMessages:
			str := C.GoString((*C.char)(rosMsg.Data))
			ch <- msg.Message{
				Timestamp:   time.Now().UTC(),
				From:        "fleet",
				To:          droneName,
				ID:          "",
				MessageType: "tasks-assigned",
				Message:     str,
			}
		}
	}
}

// func serialize(i interface{}) string {
// 	b, err := json.Marshal(i)
// 	if err != nil {
// 		panic(err)
// 	}

// 	return string(b)
// }

// func deserialize(jsonString string, i interface{}) {
// 	err := json.Unmarshal([]byte(jsonString), i)
// 	if err != nil {
// 		panic(err)
// 	}
// }
