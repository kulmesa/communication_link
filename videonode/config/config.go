package config

import (
	"gopkg.in/yaml.v2"
	"log"
	"os"
)

var defaultConfig = []byte(`
default: simulator
cameras:
- id: simulator
  source: udpsrc port=5600`)

type Config struct {
	Default string `yaml:"default"`
	Cameras []struct {
		Id         string `yaml:"id"`
		Source     string `yaml:"source"`
		Parameters string `yaml:"parameters"`
	} `yaml:"cameras"`
}

// NewConfig returns a new decoded Config struct
func NewConfig(configPath string) *Config {
	config := &Config{}
	file, err := os.Open(configPath)
	if err != nil {
		log.Println("Config file not found, using default values")
		yaml.Unmarshal(defaultConfig, &config)
		return config
	}
	defer file.Close()
	d := yaml.NewDecoder(file)
	if err := d.Decode(&config); err != nil {
		log.Println("Config file decode error, using default values")
		yaml.Unmarshal(defaultConfig, &config)
		return config
	}
	return config
}

func (c Config) GetSource(s string) string {
	defSource := ""
	for _, cam := range c.Cameras {
		if cam.Id == s {
			return cam.Source
		}
		if cam.Id == c.Default {
			defSource = cam.Source
		}
	}
	return defSource
}

func (c Config) GetParameters(s string) string {
	defParams := ""
	for _, cam := range c.Cameras {
		if cam.Id == s {
			return cam.Parameters
		}
		if cam.Id == c.Default {
			defParams = cam.Parameters
		}
	}
	return defParams
}
