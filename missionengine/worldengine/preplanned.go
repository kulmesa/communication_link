package worldengine

import (
	"encoding/json"
	"io/ioutil"
	"log"
)

type PreplannedPoint struct {
	X float64 `json:"lat"`
	Y float64 `json:"lon"`
	Z float64 `json:"alt"`
}

func loadPrelanned() ([]PreplannedPoint, error) {
	text, err := ioutil.ReadFile("./flightpath.json")
	if err != nil {
		log.Printf("Failed to read flightpath.json: %v", err)
		return nil, err
	}

	var path []PreplannedPoint
	err = json.Unmarshal(text, &path)
	if err != nil {
		log.Printf("Failed to unmarshal flightpath.json: %v", err)
		return nil, err
	}

	return path, nil
}
