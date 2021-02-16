package worldengine

import (
	"encoding/json"
	"time"

	"github.com/tiiuae/communication_link/pkg/missionengine/types"
	"github.com/tiiuae/communication_link/pkg/ros"
	rosTypes "github.com/tiiuae/communication_link/pkg/types"
)

type Drones map[string]*droneState
type Backlog []*backlogItem

type worldState struct {
	MyName     string
	LeaderName string
	Backlog    Backlog
	Drones     Drones
}

type droneState struct {
	Name   string
	Leader bool
	Tasks  []*taskState
}

type taskState struct {
	ID string
}

type backlogItem struct {
	ID     string
	Status string
	X      float64
	Y      float64
	Z      float64
}

func createState(me string) *worldState {
	return &worldState{
		MyName:     me,
		LeaderName: "",
		Backlog:    make([]*backlogItem, 0),
		Drones:     make(map[string]*droneState, 0),
	}
}

// func createDrones(fleet []string, leader string) map[string]*droneState {
// 	drones := make(map[string]*droneState, 0)
// 	for _, x := range fleet {
// 		drones[x] = &droneState{x, x == leader, []*taskState{}}
// 	}
// 	return drones
// }

func (s *worldState) handleDroneAdded(msg DroneAdded) []types.Message {
	name := msg.Name
	isLeader := len(s.Drones) == 0
	if isLeader {
		s.LeaderName = name
	}
	s.Drones[name] = &droneState{name, isLeader, []*taskState{}}

	// Done if not leader
	if s.LeaderName != s.MyName {
		return []types.Message{}
	}

	if len(s.Backlog) == 0 {
		return []types.Message{}
	}

	// Re-assign tasks
	return s.assignTasks()
}

func (s *worldState) handleTaskCreated(msg TaskCreated) []types.Message {
	// Add task to backlog
	bi := backlogItem{
		ID:     msg.ID,
		Status: "",
		X:      msg.Payload.X,
		Y:      msg.Payload.Y,
		Z:      msg.Payload.Z,
	}
	s.Backlog = append(s.Backlog, &bi)

	// Done if not leader
	if s.LeaderName != s.MyName {
		return []types.Message{}
	}

	return s.assignTasks()
}

func (s *worldState) handleTasksAssigned(msg TasksAssigned, pubPath *ros.Publisher, pubMavlink *ros.Publisher) []types.Message {
	for droneName, tasks := range msg.Tasks {
		drone := s.Drones[droneName]
		drone.Tasks = make([]*taskState, 0)
		for _, t := range tasks {
			drone.Tasks = append(drone.Tasks, &taskState{ID: t.ID})
		}
	}

	// Send PX4 path messages
	sendFlyToMessages(msg.Tasks[s.MyName], pubPath, pubMavlink)

	return []types.Message{}
}

func sendFlyToMessages(tasks []*TaskAssignment, pubPath *ros.Publisher, pubMavlink *ros.Publisher) {
	if len(tasks) == 0 {
		return
	}
	points := make([]rosTypes.Point, 0)
	for _, t := range tasks {
		points = append(points, rosTypes.Point{t.X, t.Y, t.Z})
	}

	path := rosTypes.NewPath(points)
	pubPath.DoPublish(path)

	time.Sleep(200 * time.Millisecond)
	pubMavlink.DoPublish(rosTypes.GenerateString("start_mission"))
}

func (s *worldState) handleTaskCompleted(msg TaskCompleted) []types.Message {
	for _, bi := range s.Backlog {
		if bi.ID == msg.ID {
			bi.Status = "done"
			break
		}
	}

	return []types.Message{}
}

func (s *worldState) assignTasks() []types.Message {
	drones := s.Drones.names()
	result := TasksAssigned{
		Tasks: make(map[string][]*TaskAssignment),
	}
	for i, bi := range s.Backlog {
		if bi.Status == "done" {
			continue
		}
		drone := drones[i%len(drones)]
		task := TaskAssignment{
			ID: bi.ID,
			X:  bi.X,
			Y:  bi.Y,
			Z:  bi.Z,
		}

		result.Tasks[drone] = append(result.Tasks[drone], &task)
	}

	msg1 := serialize(result)
	var temp TasksAssignedMqtt
	deserialize(msg1, &temp)
	msg2 := serialize(temp.Tasks)
	return []types.Message{
		{
			Timestamp:   time.Now().UTC(),
			From:        s.MyName,
			To:          "*",
			ID:          "id1",
			MessageType: "tasks-assigned",
			Message:     msg1,
		},
		{
			Timestamp:   time.Now().UTC(),
			From:        s.MyName,
			To:          "*",
			ID:          "id1",
			MessageType: "mission-plan",
			Message:     msg2,
		},
	}
}

func (drones *Drones) selectDrone() *droneState {
	var d *droneState
	for _, v := range *drones {
		if d == nil {
			d = v
			continue
		}
		if v.rank() < d.rank() {
			d = v
		}
	}

	return d
}

func (drones *Drones) names() []string {
	result := make([]string, 0)
	for k := range *drones {
		result = append(result, k)
	}

	return result
}

func (drone *droneState) rank() int {
	boolToInt := map[bool]int{false: 0, true: 1}
	return len(drone.Tasks) + boolToInt[drone.Leader]
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
