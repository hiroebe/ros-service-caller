package main

import (
	"io/ioutil"
	"os"
	"path/filepath"

	"gopkg.in/yaml.v2"
)

type config struct {
	Editor    string `yaml:"editor"`
	SelectCmd string `yaml:"selectcmd"`
}

func (c *config) load() {
	file := filepath.Join(os.Getenv("HOME"), ".config", "ros-service-caller", "config.yaml")
	_, err := os.Stat(file)
	if err != nil {
		return
	}
	f, err := ioutil.ReadFile(file)
	if err != nil {
		return
	}
	_ = yaml.Unmarshal(f, c)

	return
}
