package main

import (
	"context"
	"crypto/rand"
	"crypto/rsa"
	"crypto/x509"
	"encoding/json"
	"encoding/pem"
	"fmt"
	"io/ioutil"
	"log"
	"os"
	"strings"
	"sync"
	"time"

	mqtt "github.com/eclipse/paho.mqtt.golang"
	"github.com/tiiuae/communication_link/missionengine"
	ros "github.com/tiiuae/communication_link/ros"
	types "github.com/tiiuae/communication_link/types"
	"golang.org/x/crypto/ssh"
	"golang.org/x/crypto/ssh/knownhosts"
)

var missionSlug string = ""

type controlCommand struct {
	Command   string
	Payload   string
	Timestamp time.Time
}
type gstreamerCmd struct {
	Command   string
	Address   string
	Timestamp time.Time
}

type trustEvent struct {
	PublicSSHKey string `json:"public_ssh_key"`
}

type missionEvent struct {
	MissionSlug string    `json:"mission_slug"`
	Timestamp   time.Time `json:"timestamp"`
}

func InitializeTrust(client mqtt.Client) {
	//publicKey, privateKey, err := ed25519.GenerateKey(nil)
	privateKey, err := rsa.GenerateKey(rand.Reader, 4096)
	publicKey := &privateKey.PublicKey
	if err != nil {
		log.Print("Could not generate keys")
		return
	}

	privBytes := x509.MarshalPKCS1PrivateKey(privateKey)

	privatePemData := pem.EncodeToMemory(&pem.Block{
		Type:  "RSA PRIVATE KEY",
		Bytes: privBytes,
	})

	os.Mkdir("ssh", 0755)
	err = ioutil.WriteFile("ssh/id_rsa", privatePemData, 0600)

	sshPublicKey, _ := ssh.NewPublicKey(publicKey)
	sshPublicKeyStr := ssh.MarshalAuthorizedKey(sshPublicKey)

	trust, _ := json.Marshal(trustEvent{
		PublicSSHKey: strings.TrimSuffix(string(sshPublicKeyStr), "\n"),
	})

	// send public key to server
	topic := fmt.Sprintf("/devices/%s/events/trust", *deviceID)
	tok := client.Publish(topic, qos, retain, trust)
	if !tok.WaitTimeout(10 * time.Second) {
		log.Printf("Could not send trust within 10s")
		return
	}
	err = tok.Error()
	if err != nil {
		log.Printf("Could not send trust: %v", err)
		return
	}
	log.Printf("Trust initialized")
}

func JoinMission(client mqtt.Client, payload []byte, me *missionengine.MissionEngine) {
	var info struct {
		GitServerAddress string `json:"git_server_address"`
		GitServerKey     string `json:"git_server_key"`
		MissionSlug      string `json:"mission_slug"`
	}
	err := json.Unmarshal(payload, &info)
	if err != nil {
		log.Printf("Could not unmarshal payload: %v", err)
		return
	}
	log.Printf("Git config: %+v", info)

	// TODO: knownhosts.HashHostname
	knownCloudHost := fmt.Sprintf("%s %s\n", knownhosts.Normalize(info.GitServerAddress), info.GitServerKey)
	err = ioutil.WriteFile("ssh/known_host_cloud", []byte(knownCloudHost), 0644)
	if err != nil {
		log.Printf("Could not write known_host_cloud file: %v", err)
		return
	}

	missionSlug = info.MissionSlug

	sshUrl := fmt.Sprintf("ssh://git@%s/%s.git", info.GitServerAddress, info.MissionSlug)
	me.Start(sshUrl, info.GitServerKey)
}

func LeaveMission(client mqtt.Client, payload []byte, me *missionengine.MissionEngine) {
	missionSlug = ""
	// me.Stop()
}

func UpdateBacklog(me *missionengine.MissionEngine) {
	me.UpdateBacklog()
}

// handleControlCommand takes a command string and forwards it to mavlinkcmd
func handleControlCommand(command string, mqttClient mqtt.Client, pub *ros.Publisher, me *missionengine.MissionEngine) {
	var cmd controlCommand
	err := json.Unmarshal([]byte(command), &cmd)
	if err != nil {
		log.Printf("Could not unmarshal command: %v", err)
		return
	}

	switch cmd.Command {
	case "initialize-trust":
		log.Printf("Initializing trust with backend")
		InitializeTrust(mqttClient)
	case "join-mission":
		log.Printf("Backend requesting to join a mission")
		JoinMission(mqttClient, []byte(cmd.Payload), me)
	case "leave-mission":
		log.Printf("Backend requesting to leave from mission")
		LeaveMission(mqttClient, []byte(cmd.Payload), me)
	case "update-backlog":
		log.Printf("Backend requesting to update backlog")
		UpdateBacklog(me)
	case "takeoff":
		log.Printf("Publishing 'takeoff' to /mavlinkcmd")
		pub.DoPublish(types.GenerateString("takeoff"))
	case "land":
		log.Printf("Publishing 'land' to /mavlinkcmd")
		pub.DoPublish(types.GenerateString("land"))
	case "start_mission":
		log.Printf("Publishing 'start_mission' to /mavlinkcmd")
		pub.DoPublish(types.GenerateString("start_mission"))
	case "pause_mission":
		log.Printf("Publishing 'pause_mission' to /mavlinkcmd")
		pub.DoPublish(types.GenerateString("pause_mission"))
	case "resume_mission":
		log.Printf("Publishing 'resume_mission' to /mavlinkcmd")
		pub.DoPublish(types.GenerateString("resume_mission"))
	case "return_home":
		log.Printf("Publishing 'return_home' to /mavlinkcmd")
		pub.DoPublish(types.GenerateString("return_home"))
	//case "plan":
	//	log.Printf("Publishing 'plan' to /mavlinkcmd")
	//	msg.SetText("plan")
	//	err := pub.Publish(msg.GetMessage(), msg.GetData())
	//	if err != nil {
	//		log.Fatalf("Publish failed: %v", err)
	//	}
	default:
		log.Printf("Unknown command: %v", command)
	}
}

// handleMissionCommand takes a command string and forwards it to mavlinkcmd
func handleMissionCommand(command string, pub *ros.Publisher) {
	var cmd controlCommand
	err := json.Unmarshal([]byte(command), &cmd)
	if err != nil {
		log.Printf("Could not unmarshal command: %v", err)
		return
	}
	switch cmd.Command {
	case "new_mission":
		log.Printf("Publishing mission to where ever")
		//		mission := new (types.PoseStamped)
		pub.DoPublish(types.GeneratePath())
	default:
		log.Printf("Unknown command: %v", command)
	}
}

// handleGstreamerCommand takes a command string and forwards it to gstreamercmd
func handleGstreamerCommand(command string, pub *ros.Publisher) {
	var cmd gstreamerCmd

	err := json.Unmarshal([]byte(command), &cmd)
	if err != nil {
		log.Printf("%v\nCould not unmarshal command: %v", command, err)
		return
	}
	switch cmd.Command {
	case "start":
		log.Printf("Publishing 'start' to /videostreamrcmd")
		pub.DoPublish(types.GenerateString(command))
	case "stop":
		log.Printf("Publishing 'stop' to /videostreamrcmd")
		pub.DoPublish(types.GenerateString(command))
	default:
		log.Printf("Unknown command: %v", command)
	}
}

// handleControlCommands routine waits for commands and executes them. The routine quits when quit channel is closed
func handleControlCommands(ctx context.Context, wg *sync.WaitGroup, mqttClient mqtt.Client, node *ros.Node, commands <-chan string, me *missionengine.MissionEngine) {
	wg.Add(1)
	defer wg.Done()
	pub := node.InitPublisher("mavlinkcmd", "std_msgs/msg/String", (*types.String)(nil))
	for {
		select {
		case <-ctx.Done():
			pub.Finish()
			return
		case command := <-commands:
			handleControlCommand(command, mqttClient, pub, me)
		}
	}
}

// handleMissionCommands routine waits for commands and executes them. The routine quits when quit channel is closed
func handleMissionCommands(ctx context.Context, wg *sync.WaitGroup, node *ros.Node, commands <-chan string) {
	wg.Add(1)
	defer wg.Done()
	pub := node.InitPublisher("whereever", "nav_msgs/msg/Path", (*types.Path)(nil))
	for {
		select {
		case <-ctx.Done():
			pub.Finish()
			return
		case command := <-commands:
			handleMissionCommand(command, pub)
		}
	}
}

// handleGstreamerCommands routine waits for commands and executes them. The routine quits when quit channel is closed
func handleGstreamerCommands(ctx context.Context, wg *sync.WaitGroup, node *ros.Node, commands <-chan string) {
	wg.Add(1)
	defer wg.Done()
	pub := node.InitPublisher("videostreamcmd", "std_msgs/msg/String", (*types.String)(nil))
	for {
		select {
		case <-ctx.Done():
			pub.Finish()
			return
		case command := <-commands:
			handleGstreamerCommand(command, pub)
		}
	}
}

func publishMissionState(ctx context.Context, wg *sync.WaitGroup, mqttClient mqtt.Client) {
	wg.Add(1)
	defer wg.Done()
	for {
		select {
		case <-ctx.Done():
			return
		case <-time.After(15 * time.Second):
			topic := fmt.Sprintf("/devices/%s/events/mission-state", *deviceID)
			msg := missionEvent{
				MissionSlug: missionSlug,
				Timestamp:   time.Now().UTC(),
			}
			b, _ := json.Marshal(msg)
			mqttClient.Publish(topic, 1, false, b)
		}
	}
}

func startCommandHandlers(ctx context.Context, wg *sync.WaitGroup, mqttClient mqtt.Client, node *ros.Node, me *missionengine.MissionEngine) {

	controlCommands := make(chan string)
	missionCommands := make(chan string)
	gstreamerCommands := make(chan string)

	go handleControlCommands(ctx, wg, mqttClient, node, controlCommands, me)
	go handleMissionCommands(ctx, wg, node, missionCommands)
	go handleGstreamerCommands(ctx, wg, node, gstreamerCommands)
	go publishMissionState(ctx, wg, mqttClient)

	log.Printf("Subscribing to MQTT commands")
	commandTopic := fmt.Sprintf("/devices/%s/commands/", *deviceID)
	token := mqttClient.Subscribe(fmt.Sprintf("%v#", commandTopic), 0, func(client mqtt.Client, msg mqtt.Message) {
		subfolder := strings.TrimPrefix(msg.Topic(), commandTopic)
		switch subfolder {
		case "control":
			log.Printf("Got control command: %v", string(msg.Payload()))
			controlCommands <- string(msg.Payload())
		case "mission":
			log.Printf("Got mission command")
			missionCommands <- string(msg.Payload())
		case "videostream":
			log.Printf("Got videostream command")
			gstreamerCommands <- string(msg.Payload())
		default:
			log.Printf("Unknown command subfolder: %v", subfolder)
		}
	})
	if err := token.Error(); err != nil {
		log.Fatalf("Error on subscribe: %v", err)
	}
}
