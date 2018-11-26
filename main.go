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
	topic      = topicCmd.Arg("topic", "ROS topic name").String()
	pubOnce    = topicCmd.Flag("once", "Enable once mode").Short('1').Bool()

	modeService = Mode{
		name:      "service",
		node:      service,
		protoMode: "srv",
		rosCmd:    "rosservice",
	}
	modeTopic = Mode{
		name:      "topic",
		node:      topic,
		protoMode: "msg",
		rosCmd:    "rostopic",
	}
)

type Mode struct {
	name      string
	node      *string
	protoMode string
	rosCmd    string
}

func (m *Mode) selectNode(selectcmd string) error {
	f, err := ioutil.TempFile("", "")
	if err != nil {
		return err
	}
	defer os.Remove(f.Name())
	defer f.Close()

	err = execCommand(fmt.Sprintf("%s list | %s > %s", m.rosCmd, selectcmd, f.Name()))
	if err != nil {
		return err
	}
	node, err := readDataFromFile(f)
	if err != nil {
		return err
	}
	*m.node = node
	return nil
}

func (m *Mode) getProto(target string) ([]byte, error) {
	content, err := execCommandOutput(fmt.Sprintf("rosmsg-proto %s %s", m.protoMode, target))
	if err != nil {
		return nil, err
	}
	content = bytes.Trim(content, `"`)
	return content, nil
}

func (m *Mode) action() func(c *kingpin.ParseContext) error {
	return func(c *kingpin.ParseContext) error {
		var err error

		if err = checkRoscoreRunning(); err != nil {
			return err
		}

		if *m.node == "" {
			if cfg.SelectCmd == "" {
				fmt.Println("selectcmd is not set.")
				return nil
			}
			err = m.selectNode(cfg.SelectCmd)
			if err != nil {
				return err
			}
		}

		var argsType string
		t, err := execCommandOutput(fmt.Sprintf("%s type %s", m.rosCmd, *m.node))
		if err != nil {
			return err
		}
		argsType = string(t)

		var args string
		if *datafile != nil {
			args, err = readDataFromFile(*datafile)
			if err != nil {
				return err
			}
		} else {
			proto, err := m.getProto(argsType)
			if err != nil {
				return err
			}
			args, err = editArgs(proto)
			if err != nil {
				return err
			}
		}

		if m.name == "service" {
			err = callService(*m.node, args)
		} else if m.name == "topic" {
			err = pubTopic(*m.node, argsType, args)
		}
		return err
	}
}

func checkRoscoreRunning() error {
	if err := execCommand("rosnode list 1>/dev/null"); err != nil {
		return err
	}
	return nil
}

func editArgs(proto []byte) (string, error) {
	f, err := ioutil.TempFile("", "*.yaml")
	if err != nil {
		return "", err
	}
	filename := f.Name()
	defer os.Remove(f.Name())

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

func callService(service, args string) error {
	cmd := fmt.Sprintf(`rosservice call %s "%s"`, service, args)
	fmt.Println("--- IN ---")
	fmt.Println(cmd)
	fmt.Println("--- OUT ---")
	return execCommand(cmd)
}

func pubTopic(topic, msg, args string) error {
	onceFlag := ""
	if *pubOnce {
		onceFlag = "-1"
	}
	cmd := fmt.Sprintf(`rostopic pub %s %s %s "%s"`, onceFlag, topic, msg, args)
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

func main() {
	app.Version(version)
	app.HelpFlag.Short('h')
	serviceCmd.Action(modeService.action())
	topicCmd.Action(modeTopic.action())

	cfg.load()

	kingpin.MustParse(app.Parse(os.Args[1:]))
}
