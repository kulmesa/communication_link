package worldengine

import (
	"encoding/json"
	"log"

	"github.com/tiiuae/communication_link/missionengine/types"
	"github.com/tiiuae/communication_link/ros"
)

type WorldEngine struct {
	Me     string
	Leader string
	Fleet  []string
	state  *worldState
}

func New(me string) *WorldEngine {
	return &WorldEngine{me, "", make([]string, 0), createState(me)}
}

func (we *WorldEngine) HandleMessage(msg types.Message, pubPath *ros.Publisher, pubMavlink *ros.Publisher) []types.Message {
	state := we.state
	outgoing := make([]types.Message, 0)
	log.Printf("WorldEngine handling: %s (%s -> %s)", msg.MessageType, msg.From, msg.To)
	if msg.MessageType == "drone-added" {
		var message DroneAdded
		json.Unmarshal([]byte(msg.Message), &message)
		we.Fleet = append(we.Fleet, message.Name)
		if len(we.Fleet) == 1 {
			we.Leader = message.Name
		}
		outgoing = state.handleDroneAdded(message)
	} else if msg.MessageType == "task-created" && we.Me == we.Leader {
		var message TaskCreated
		json.Unmarshal([]byte(msg.Message), &message)
		if message.Type == "fly-to" {
			var inner FlyToTaskCreated
			json.Unmarshal([]byte(msg.Message), &inner)
			outgoing = state.handleFlyToTaskCreated(inner)
		} else if message.Type == "execute-preplanned" {
			var inner ExecutePredefinedTaskCreated
			json.Unmarshal([]byte(msg.Message), &inner)
			outgoing = state.handleExecutePredefinedToTaskCreated(inner)
		}
	} else if msg.MessageType == "tasks-assigned" {
		var message TasksAssigned
		json.Unmarshal([]byte(msg.Message), &message)
		outgoing = state.handleTasksAssigned(message, pubPath, pubMavlink)
	} else if msg.MessageType == "task-completed" {
		var message TaskCompleted
		json.Unmarshal([]byte(msg.Message), &message)
		outgoing = state.handleTaskCompleted(message)
	}

	return outgoing
}

// Message types

type DroneAdded struct {
	Name string `json:"name"`
}
type TaskCreated struct {
	ID   string `json:"id"`
	Type string `json:"type"`
}

type FlyToTaskCreated struct {
	ID      string `json:"id"`
	Type    string `json:"type"`
	Payload struct {
		X float64 `json:"lat"`
		Y float64 `json:"lon"`
		Z float64 `json:"alt"`
	} `json:"payload"`
}

type ExecutePredefinedTaskCreated struct {
	ID      string `json:"id"`
	Type    string `json:"type"`
	Payload struct {
		Drone string `json:"drone"`
	} `json:"payload"`
}

type TasksAssigned struct {
	Tasks map[string][]*TaskAssignment `json:"tasks"`
}

type TaskAssignment struct {
	ID   string  `json:"id"`
	Type string  `json:"type"`
	X    float64 `json:"lat"`
	Y    float64 `json:"lon"`
	Z    float64 `json:"alt"`
}

type TasksAssignedMqtt struct {
	Tasks map[string][]*TaskAssignmentMqtt `json:"tasks"`
}

type TaskAssignmentMqtt struct {
	ID string `json:"id"`
}

type TaskCompleted struct {
	ID string `json:"id"`
}
