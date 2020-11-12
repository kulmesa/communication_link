package main

import (
	"crypto/tls"
	"encoding/json"
	"flag"
	"fmt"
	"io/ioutil"
	"log"
	"os"
	"os/signal"
	"strings"
	"syscall"
	"time"

	jwt "github.com/dgrijalva/jwt-go"
	mqtt "github.com/eclipse/paho.mqtt.golang"
	"github.com/ssrc-tii/rclgo"
	"github.com/ssrc-tii/rclgo/types"
)

const (
	RegistryID     = "fleet-registry"
	ProjectID      = "auto-fleet-mgnt"
	Region         = "europe-west1"
	PrivateKeyPath = "rsa_private.pem"
	Algorithm      = "RS256"
	Server         = "ssl://mqtt.googleapis.com:8883"
)

var (
	DeviceID = flag.String("device_id", "", "The provisioned device id")
)

// MQTT parameters
const (
	TopicType = "events" // or "state"
	QoS       = 1        // QoS 2 isn't supported in GCP
	Retain    = false
	Username  = "unused" // always this value in GCP
)

type ControlCommand struct {
	Command   string
	Timestamp time.Time
}

// handleControlCommand takes a command string and forwards it to mavlinkcmd
func handleControlCommand(command string, msg types.StdMsgsString, pub rclgo.Publisher) {
	var cmd ControlCommand
	err := json.Unmarshal([]byte(command), &cmd)
	if err != nil {
		log.Printf("Could not unmarshal command: %v", err)
		return
	}

	switch cmd.Command {
	case "takeoff":
		log.Printf("Publishing 'takeoff' to /mavlinkcmd")
		msg.SetText("takeoff")
		err := pub.Publish(msg.GetMessage(), msg.GetData())
		if err != nil {
			log.Fatalf("Publish failed: %v", err)
		}
	case "land":
		log.Printf("Publishing 'land' to /mavlinkcmd")
		msg.SetText("land")
		err := pub.Publish(msg.GetMessage(), msg.GetData())
		if err != nil {
			log.Fatalf("Publish failed: %v", err)
		}
	default:
		log.Printf("Unknown command: %v", command)
	}
}

// handleControlCommands routine waits for commands and executes them. The routine quits when quit channel is closed
func handleControlCommands(node rclgo.Node, commands <-chan string, quit <-chan struct{}) {
	pub := rclgo.NewZeroInitializedPublisher()
	pubOpts := rclgo.NewPublisherDefaultOptions()

	var msg types.StdMsgsString
	msg.InitMessage()
	defer msg.DestroyMessage()

	log.Println("Creating the publisher")

	err := pub.Init(pubOpts, node, "/mavlinkcmd", msg.GetMessage())
	if err != nil {
		log.Fatalf("Could not initialide publisher: %v", err)
	}
	defer func() {
		err = pub.PublisherFini(node)
		if err != nil {
			log.Fatalf("Could not finalize subscription: %v", err)
		}
	}()

	for {
		select {
		case command := <-commands:
			handleControlCommand(command, msg, pub)
		case <-quit:
			return
		}
	}

}


func handleGpsMesssages(node rclgo.Node, quit <-chan struct{}, mgttClient mqtt.Client){

	msg := make(chan string, 1)
	go func() {
		// Receive input in a loop
		for {
			var s string
			fmt.Scan(&s)
			// Send what we read over the channel
			msg <- s
		}
	}()

	//Create the subscriptor
	mySub := rclgo.NewZeroInitializedSubscription()
	mySubOpts := rclgo.NewSubscriptionDefaultOptions()

	//Creating the type
//	msgType := types.GetMessageTypeFromStdMsgsString()
	msgType := types.GetMessageTypeFromStdMsgsString()
	

	fmt.Printf("Creating the subscriber! \n")
	err := mySub.Init(mySubOpts, node, "/VehicleGlobalPosition_temp", msgType)
	if err != nil {
		log.Fatalf("SubscriptionsInit: %s", err)
	}

	//Creating the msg type
	var myMsg types.StdMsgsString
	myMsg.InitMessage()

//loop:
	for {

		//fmt.Printf("(Suscriber loop 1\n")
		err = mySub.TakeMessage(&myMsg.MsgInfo, myMsg.GetData())
		//fmt.Printf("TakeMessage err %s\n", err)
		if err == nil {
//			fmt.Printf("(Suscriber) Received %s\n", myMsg.GetDataAsString())
			pubmes := fmt.Sprintf("{Coordinates:%s}", myMsg.GetDataAsString())
			fmt.Printf(pubmes)
			//mgttClient.Publish("/testing", 0, false, pubmes)
		}
		//fmt.Printf("(Suscriber loop 2\n")


		time.Sleep(100 * time.Millisecond)
/*		select {
		case <-quit:
			fmt.Println("Got shutdown, exiting")
			break loop
		case <-msg:
			fmt.Println("kukkuu")
		}*/
	}

	fmt.Printf("Shutting down!! \n")

	myMsg.DestroyMessage()
	err = mySub.SubscriptionFini(node)
	if err != nil {
		log.Fatalf("SubscriptionFini: %s", err)
	}

}



func main() {
	flag.Parse()
	// attach sigint & sigterm listeners
	terminationSignals := make(chan os.Signal, 1)
	signal.Notify(terminationSignals, syscall.SIGINT, syscall.SIGTERM)

	quit := make(chan struct{})
	go func() {
		// wait for termination and close quit to signal all
		<-terminationSignals
		log.Printf("Shuttding down..")
		close(quit)
	}()

	// create context
	ctx := rclgo.NewZeroInitializedContext()
	err := ctx.Init()
	if err != nil {
		log.Fatalf("Could not initialize context: %v", err)
	}
	log.Println("Context initialized: test")

	node := rclgo.NewZeroInitializedNode()
	nodeOpts := rclgo.NewNodeDefaultOptions()

	log.Println("Creating the node")

	err = node.Init("GoPublisher", "", ctx, nodeOpts)
	if err != nil {
		log.Fatalf("Could not initialize node: %v", err)
	}

	controlCommands := make(chan string)

	// start the control command handler
	go handleControlCommands(node, controlCommands, quit)

	mqttClient := createMQTTClient()
	defer mqttClient.Disconnect(1000)


	// start the gps message listener
	go handleGpsMesssages(node, quit, mqttClient)


	log.Printf("Wait for mqtt messages..")
	commandTopic := fmt.Sprintf("/devices/%s/commands/", *DeviceID)
	token := mqttClient.Subscribe(fmt.Sprintf("%v#", commandTopic), 0, func(client mqtt.Client, msg mqtt.Message) {
		subfolder := strings.TrimPrefix(msg.Topic(), commandTopic)
		switch subfolder {
		case "control":
			log.Printf("Got control command: %v", string(msg.Payload()))
			controlCommands <- string(msg.Payload())
		case "mission":
			log.Printf("Got mission command, not yet supported")
		default:
			log.Printf("Unknown command subfolder: %v", subfolder)
		}
	})
	if err := token.Error(); err != nil {
		log.Fatalf("Error on subscribe: %v", err)
	}

	// wait for quit signal
	<-quit

	err = node.Fini()
	if err != nil {
		log.Fatalf("Could not finalize node: %v", err)
	}

	err = ctx.Shutdown()
	if err != nil {
		log.Fatalf("Shutdown not successfull: %v", err)
	}
}

func createMQTTClient() mqtt.Client {

	// generate MQTT client
	clientID := fmt.Sprintf(
		"projects/%s/locations/%s/registries/%s/devices/%s",
		ProjectID, Region, RegistryID, *DeviceID)

	log.Println("Client ID:", clientID)

	// load private key
	keyData, err := ioutil.ReadFile(PrivateKeyPath)
	if err != nil {
		panic(err)
	}

	var key interface{}
	switch Algorithm {
	case "RS256":
		key, err = jwt.ParseRSAPrivateKeyFromPEM(keyData)
	case "ES256":
		key, err = jwt.ParseECPrivateKeyFromPEM(keyData)
	default:
		log.Fatalf("Unknown algorithm: %s", Algorithm)
	}
	if err != nil {
		panic(err)
	}

	// generate JWT as the MQTT password
	t := time.Now()
	token := jwt.NewWithClaims(jwt.GetSigningMethod(Algorithm), &jwt.StandardClaims{
		IssuedAt:  t.Unix(),
		ExpiresAt: t.Add(time.Minute * 120).Unix(),
		Audience:  ProjectID,
	})
	pass, err := token.SignedString(key)
	if err != nil {
		panic(err)
	}

	// configure MQTT client
	opts := mqtt.NewClientOptions().
		AddBroker(Server).
		SetClientID(clientID).
		SetUsername(Username).
		SetTLSConfig(&tls.Config{MinVersion: tls.VersionTLS12}).
		SetPassword(pass).
		SetProtocolVersion(4) // Use MQTT 3.1.1

	client := mqtt.NewClient(opts)

	// connect to GCP Cloud IoT Core
	log.Println("Connecting...")
	tok := client.Connect()
	if err := tok.Error(); err != nil {
		panic(err)
	}
	if !tok.WaitTimeout(time.Second * 5) {
		log.Fatalf("Connection Timeout")
	}
	if err := tok.Error(); err != nil {
		panic(err)
	}

	// need mqtt reconnect each 120 minutes for long use

	return client
}
