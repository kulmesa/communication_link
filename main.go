package main

import (
	"context"
	"flag"
	"log"
	"os"
	"os/signal"
	"sync"
	"syscall"
	"time"

	// "github.com/tiiuae/communication_link/missionengine"

	ros "github.com/tiiuae/communication_link/ros"
	types "github.com/tiiuae/communication_link/types"
)

const (
	registryID    = "fleet-registry"
	projectID     = "auto-fleet-mgnt"
	region        = "europe-west1"
	algorithm     = "RS256"
	defaultServer = "ssl://mqtt.googleapis.com:8883"
)

var (
	deviceID = flag.String("device_id", "", "The provisioned device id")
	// mqttBrokerAddress = flag.String("mqtt_broker", "", "MQTT broker protocol, address and port")
	// privateKeyPath    = flag.String("private_key", "/enclave/rsa_private.pem", "The private key for the MQTT authentication")
)

// MQTT parameters
const (
	TopicType = "events" // or "state"
	QoS       = 1        // QoS 2 isn't supported in GCP
	Retain    = false
	Username  = "unused" // always this value in GCP
)

func main() {
	flag.Parse()
	// attach sigint & sigterm listeners
	terminationSignals := make(chan os.Signal, 1)
	signal.Notify(terminationSignals, syscall.SIGINT, syscall.SIGTERM)

	// quitFunc will be called when process is terminated
	ctx, quitFunc := context.WithCancel(context.Background())

	// wait group will make sure all goroutines have time to clean up
	var wg sync.WaitGroup

	// mqttClient := newMQTTClient()
	// defer mqttClient.Disconnect(1000)

	node := ros.InitRosNode(*deviceID, "communication_link")
	defer node.ShutdownRosNode()
	// startTelemetry(ctx, &wg, mqttClient)
	// startCommandHandlers(ctx, &wg, mqttClient)

	// missionengine.Run(ctx)
	go runSubscriber(ctx, node)
	go runPublisher(ctx, node)

	// wait for termination and close quit to signal all
	<-terminationSignals
	// cancel the main context
	log.Printf("Shuttding down..")
	quitFunc()

	// wait until goroutines have done their cleanup
	log.Printf("Waiting for routines to finish..")
	wg.Wait()
	log.Printf("Signing off - BYE")
}

func runPublisher(ctx context.Context, node *ros.RosNode) {
	pub := node.InitPublisher("segmentation_violation", "std_msgs/msg/String", (*types.String)(nil))
	for i := 0; i < 5; i++ {
		time.Sleep(100 * time.Millisecond)
		str := types.GenerateString("hello world")
		pub.DoPublish(str)
	}
}

func runSubscriber(ctx context.Context, node *ros.RosNode) {
	messages := make(chan types.String)
	sub := node.InitSubscriber(messages, "segmentation_violation", "std_msgs/msg/String")
	go sub.DoSubscribe(ctx)

	for m := range messages {
		// str := C.GoString((*C.char)(m.Data))
		log.Printf("Received: %v", m.Size)
	}
}

// func newMQTTClient() mqtt.Client {
// 	serverAddress := *mqttBrokerAddress
// 	if serverAddress == "" {
// 		serverAddress = defaultServer
// 	}
// 	log.Printf("address: %v", serverAddress)

// 	// generate MQTT client
// 	clientID := fmt.Sprintf(
// 		"projects/%s/locations/%s/registries/%s/devices/%s",
// 		projectID, region, registryID, *deviceID)

// 	log.Println("Client ID:", clientID)

// 	// load private key
// 	keyData, err := ioutil.ReadFile(*privateKeyPath)
// 	if err != nil {
// 		panic(err)
// 	}

// 	var key interface{}
// 	switch algorithm {
// 	case "RS256":
// 		key, err = jwt.ParseRSAPrivateKeyFromPEM(keyData)
// 	case "ES256":
// 		key, err = jwt.ParseECPrivateKeyFromPEM(keyData)
// 	default:
// 		log.Fatalf("Unknown algorithm: %s", algorithm)
// 	}
// 	if err != nil {
// 		panic(err)
// 	}

// 	// generate JWT as the MQTT password
// 	t := time.Now()
// 	token := jwt.NewWithClaims(jwt.GetSigningMethod(algorithm), &jwt.StandardClaims{
// 		IssuedAt:  t.Unix(),
// 		ExpiresAt: t.Add(time.Minute * 120).Unix(),
// 		Audience:  projectID,
// 	})
// 	pass, err := token.SignedString(key)
// 	if err != nil {
// 		panic(err)
// 	}

// 	// configure MQTT client
// 	opts := mqtt.NewClientOptions().
// 		AddBroker(serverAddress).
// 		SetClientID(clientID).
// 		SetUsername(Username).
// 		SetTLSConfig(&tls.Config{MinVersion: tls.VersionTLS12}).
// 		SetPassword(pass).
// 		SetProtocolVersion(4) // Use MQTT 3.1.1

// 	client := mqtt.NewClient(opts)

// 	// connect to GCP Cloud IoT Core
// 	log.Printf("Connecting MQTT...")
// 	tok := client.Connect()
// 	if err := tok.Error(); err != nil {
// 		panic(err)
// 	}
// 	if !tok.WaitTimeout(time.Second * 5) {
// 		log.Fatalf("Connection Timeout")
// 	}
// 	if err := tok.Error(); err != nil {
// 		panic(err)
// 	}
// 	log.Printf("..Connected")

// 	// need mqtt reconnect each 120 minutes for long use

// 	return client
// }
