package types

import "time"

// type MessageType int

// const (
// 	MessageTypeTask MessageType = iota
// 	MessageTypeInbox
// 	MessageTypeOutbox
// 	MessageTypeUnknown
// )

type Message struct {
	// MessageSource MessageType
	Timestamp   time.Time
	From        string
	To          string
	ID          string
	MessageType string
	Message     string
}

// // GroupMessagesByType groups list of messages into
// // 1) list of new tasks from operator
// // 2) list of messages for me
// // 2) list of messages from others
// func GroupMessagesByType(messages []Message) ([]Message, []Message, []Message) {
// 	tasks := make([]Message, 0)
// 	inbox := make([]Message, 0)
// 	notifications := make([]Message, 0)
// 	for _, x := range messages {
// 		if x.MessageSource == MessageTypeTask {
// 			tasks = append(tasks, x)
// 		} else if x.MessageSource == MessageTypeInbox {
// 			inbox = append(inbox, x)
// 		} else if x.MessageSource == MessageTypeOutbox {
// 			notifications = append(notifications, x)
// 		}
// 	}
// 	return tasks, inbox, notifications
// }
