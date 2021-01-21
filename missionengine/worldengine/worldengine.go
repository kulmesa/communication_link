package worldengine

import (
	"encoding/json"
	"log"

	"github.com/tiiuae/communication_link/missionengine/types"
)

type WorldEngine struct {
	Me     string
	Leader string
	Fleet  []string
	state  *worldState
}

func New(me string, fleet []string, leader string) *WorldEngine {
	return &WorldEngine{me, leader, fleet, createState(fleet, leader, me)}
}

func (we *WorldEngine) HandleMessage(msg types.Message) []types.Message {
	state := we.state
	outgoing := make([]types.Message, 0)
	log.Printf("WorldEngine handling: %s (%s -> %s)", msg.MessageType, msg.From, msg.To)
	if msg.MessageType == "task-created" && we.Me == we.Leader {
		var message TaskCreated
		json.Unmarshal([]byte(msg.Message), &message)
		outgoing = state.handleTaskCreated(message)
	} else if msg.MessageType == "tasks-assigned" {
		var message TasksAssigned
		json.Unmarshal([]byte(msg.Message), &message)
		outgoing = state.handleTasksAssigned(message)
	} else if msg.MessageType == "task-completed" {
		var message TaskCompleted
		json.Unmarshal([]byte(msg.Message), &message)
		outgoing = state.handleTaskCompleted(message)
	}

	return outgoing
}

// Message types

type TaskCreated struct {
	ID string  `json:"id"`
	X  float64 `json:"x"`
	Y  float64 `json:"y"`
	Z  float64 `json:"z"`
}

type TasksAssigned struct {
	Tasks map[string][]*TaskAssignment `json:"tasks"`
}

type TaskAssignment struct {
	ID string  `json:"id"`
	X  float64 `json:"x"`
	Y  float64 `json:"y"`
	Z  float64 `json:"z"`
}

type TaskCompleted struct {
	ID string `json:"id"`
}
