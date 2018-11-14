package main

import (
	"bytes"
	"fmt"
	"io/ioutil"
	"os"
	"os/exec"

	"gopkg.in/alecthomas/kingpin.v2"
)

const (
	version = "0.0.1"
)

var (
	cfg config

	app        = kingpin.New("ros-service-caller", "Edit args for rosservice-call with editor")
	serviceCmd = app.Command("service", `execute "rosservice call"`)
	topicCmd   = app.Command("topic", `execute "rostopic pub"`)
	datafile   = app.Flag("file", "Load yaml file").Short('f').File()
	service    = serviceCmd.Arg("service", "ROS service name").String()
)

func action(c *kingpin.ParseContext) error {
	var err error

	if err = execCommand("rosnode list 1>/dev/null"); err != nil {
		return err
	}

	if *service == "" {
		if cfg.SelectCmd == "" {
			fmt.Println("selectcmd is not set.")
			return nil
		}
		*service, err = selectService(cfg.SelectCmd)
		if err != nil {
			return err
		}
	}

	var args string

	if *datafile != nil {
		args, err = readDataFromFile(*datafile)
	} else {
		args, err = editArgs()
	}
	if err != nil {
		return err
	}

	return callService(*service, args)
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
	return readDataFromFile(f)
}

func editArgs() (string, error) {
	f, err := ioutil.TempFile("", "*.yaml")
	if err != nil {
		return "", err
	}
	filename := f.Name()
	defer os.Remove(f.Name())

	proto, err := getSrvProto(*service)
	if err != nil {
		return "", err
	}
	_, err = f.Write(proto)
	if err != nil {
		return "", err
	}

	// Close tempfile here for editing with editor.
	f.Close()

	editor := getEditor()
	err = execCommand(editor + " " + filename)
	if err != nil {
		return "", err
	}

	return readDataFromFileName(filename)
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

func getEditor() string {
	editor := cfg.Editor
	if editor == "" {
		editor = os.Getenv("EDITOR")
	}
	if editor == "" {
		editor = "vim"
	}
	return editor
}

func callService(s, a string) error {
	cmd := fmt.Sprintf(`rosservice call %s "%s"`, s, a)
	fmt.Println("--- IN ---")
	fmt.Println(cmd)
	fmt.Println("--- OUT ---")
	return execCommand(cmd)
}

func readDataFromFileName(filename string) (string, error) {
	data, err := ioutil.ReadFile(filename)
	if err != nil {
		return "", err
	}
	data = bytes.TrimRight(data, "\n")
	return string(data), nil
}

func readDataFromFile(f *os.File) (string, error) {
	data, err := ioutil.ReadAll(f)
	if err != nil {
		return "", err
	}
	data = bytes.TrimRight(data, "\n")
	return string(data), nil
}

func execCommand(command string) error {
	cmd := exec.Command("sh", "-c", command)
	cmd.Stderr = os.Stderr
	cmd.Stdout = os.Stdout
	cmd.Stdin = os.Stdin
	return cmd.Run()
}

func execCommandOutput(command string) ([]byte, error) {
	out, err := exec.Command("sh", "-c", command).Output()
	if err != nil {
		return nil, err
	}
	return bytes.TrimRight(out, "\n"), nil
}

func serviceAction(c *kingpin.ParseContext) error {
	fmt.Println("service")
	return nil
}

func topicAction(c *kingpin.ParseContext) error {
	fmt.Println("topic")
	return nil
}

func main() {
	app.Version(version)
	app.HelpFlag.Short('h')
	serviceCmd.Action(serviceAction)
	topicCmd.Action(topicAction)
	// app.Action(action)

	cfg.load()

	kingpin.MustParse(app.Parse(os.Args[1:]))
}
