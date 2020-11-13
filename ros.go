package main

import (
	"log"

	"github.com/ssrc-tii/rclgo"
	"github.com/ssrc-tii/rclgo/types"
)

func newROSContext() (rclgo.Context, func()) {
	// create context
	rclCtx := rclgo.NewZeroInitializedContext()
	err := rclCtx.Init()
	if err != nil {
		log.Fatalf("Could not initialize context: %v", err)
	}
	log.Println("Context initialized")

	return rclCtx, func() {
		log.Println("Context shutting down")
		err = rclCtx.Shutdown()
		if err != nil {
			log.Fatalf("Shutdown not successfull: %v", err)
		}
	}
}
func newROSNode(rclCtx rclgo.Context, name string) (rclgo.Node, func()) {
	node := rclgo.NewZeroInitializedNode()
	nodeOpts := rclgo.NewNodeDefaultOptions()

	log.Println("Creating the node")
	err := node.Init(name, "", rclCtx, nodeOpts)
	if err != nil {
		log.Fatalf("Could not initialize node: %v", err)
	}

	return node, func() {
		log.Println("Node shutting down")
		err := node.Fini()
		if err != nil {
			log.Fatalf("Could not finalize node: %v", err)
		}
	}
}

func newROSSubscription(node rclgo.Node, topic string, msgType types.MessageTypeSupport) (rclgo.Subscription, func()) {
	sub := rclgo.NewZeroInitializedSubscription()
	subOpts := rclgo.NewSubscriptionDefaultOptions()

	log.Printf("Creating subscriber for %s", topic)

	err := sub.Init(subOpts, node, topic, msgType)
	if err != nil {
		log.Fatalf("Could not initialize subscription %s: %v", topic, err)
	}
	return sub, func() {
		log.Println("Subscription shutting down")
		err := sub.SubscriptionFini(node)
		if err != nil {
			log.Fatalf("Could not clean up subscription %s: %v", topic, err)
		}
	}
}
func newROSPublisher(node rclgo.Node, topic string, msgType types.MessageTypeSupport) (rclgo.Publisher, func()) {
	pub := rclgo.NewZeroInitializedPublisher()
	pubOpts := rclgo.NewPublisherDefaultOptions()

	log.Printf("Creating publisher for %s", topic)

	err := pub.Init(pubOpts, node, topic, msgType)
	if err != nil {
		log.Fatalf("Could not initialize publisher for %s: %v", topic, err)
	}
	return pub, func() {
		log.Println("Publisher shutting down")
		err = pub.PublisherFini(node)
		if err != nil {
			log.Fatalf("Could not clean up publisher for %s: %v", topic, err)
		}
	}
}
