package main

import (
	"bytes"
	"fmt"
	"io/ioutil"
	"log"
	"os"
	"os/exec"
	"path/filepath"

	"github.com/urfave/cli"
	"gopkg.in/yaml.v2"
)

const (
	version = "0.0.1"
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

func execCommand(command string) error {
	cmd := exec.Command("sh", "-c", command)
	cmd.Stderr = os.Stderr
	cmd.Stdout = os.Stdout
	cmd.Stdin = os.Stdin
	err := cmd.Run()
	return err
}

func execCommandOutput(command string) ([]byte, error) {
	out, err := exec.Command("sh", "-c", command).Output()
	if err != nil {
		return nil, err
	}
	return bytes.TrimRight(out, "\n"), nil
}

func action(c *cli.Context) error {
	var err error
	var service string
	var cfg config

	cfg.load()

	if c.NArg() == 0 {
		if cfg.SelectCmd == "" {
			fmt.Println("selectcmd is not set.")
			return nil
		}
		service, err = selectService(cfg.SelectCmd)
		if err != nil {
			return err
		}
	} else {
		service = c.Args().Get(0)
	}

	if file := c.String("file"); file != "" {
		err = callServiceFromFile(service, file)
		if err != nil {
			return err
		}
		return nil
	}

	err = actionWithTempFile(service, cfg)
	if err != nil {
		return err
	}

	return nil
}

func actionWithTempFile(service string, cfg config) error {
	f, err := ioutil.TempFile("", "*.yaml")
	if err != nil {
		return err
	}
	defer os.Remove(f.Name())
	defer f.Close()

	proto, err := getSrvProto(service)
	if err != nil {
		return err
	}
	_, err = f.Write(proto)
	if err != nil {
		return err
	}

	editor := cfg.Editor
	if editor == "" {
		editor = os.Getenv("EDITOR")
	}
	if editor == "" {
		editor = "vim"
	}
	err = execCommand(editor + " " + f.Name())
	if err != nil {
		return err
	}

	err = callServiceFromFile(service, f.Name())
	if err != nil {
		return err
	}

	return nil
}

func selectService(selectcmd string) (string, error) {
	f, err := ioutil.TempFile("", "")
	if err != nil {
		return "", err
	}
	defer os.Remove(f.Name())
	defer f.Close()

	err = execCommand(fmt.Sprintf("rosservice list | %s > %s", selectcmd, f.Name()))
	if err != nil {
		return "", err
	}
	out, err := ioutil.ReadFile(f.Name())
	if err != nil {
		return "", err
	}
	out = bytes.TrimRight(out, "\n")
	return string(out), nil
}

func callServiceFromFile(service, file string) error {
	f, err := ioutil.ReadFile(file)
	if err != nil {
		return err
	}
	f = bytes.TrimRight(f, "\n")
	cmd := fmt.Sprintf(`rosservice call %s "%s"`, service, string(f))
	fmt.Println("--- IN ---")
	fmt.Println(cmd)
	fmt.Println("--- OUT ---")
	err = execCommand(cmd)
	if err != nil {
		return err
	}
	return nil
}

func getSrvProto(service string) ([]byte, error) {
	srv, err := execCommandOutput("rosservice type " + service)
	if err != nil {
		return nil, err
	}
	content, err := execCommandOutput("rosmsg-proto srv " + string(srv))
	if err != nil {
		return nil, err
	}
	content = bytes.Trim(content, `"`)
	return content, nil
}

func main() {
	app := cli.NewApp()
	app.Name = "ros-service-caller"
	app.Usage = "Edit args for rosservice-call with editor"
	app.UsageText = "ros-service-caller [global options] <service>"
	app.Version = version
	app.Action = action
	app.Flags = []cli.Flag{
		cli.StringFlag{
			Name:  "file, f",
			Usage: "load yaml `FILE`",
		},
	}

	err := app.Run(os.Args)
	if err != nil {
		log.Fatal(err)
	}
}
