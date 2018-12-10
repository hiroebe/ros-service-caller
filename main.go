package main

import (
	"bytes"
	"fmt"
	"io/ioutil"
	"os"
	"strings"

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
	histCmd    = app.Command("history", "execute from history")
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
			listCmd := m.rosCmd + " list"
			*m.node, err = selectCmdOutput(listCmd, cfg.SelectCmd)
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

		var cmd string
		if m.name == "service" {
			cmd = buildServiceCmd(*m.node, args)
		} else if m.name == "topic" {
			cmd = buildTopicCmd(*m.node, argsType, args)
		}
		fmt.Println("--- IN ---")
		fmt.Println(cmd)
		fmt.Println("--- OUT ---")
		err = execCommand(cmd)
		if err != nil {
			return err
		}

		if cfg.HistFile != "" {
			err = appendToFile(cfg.HistFile, strings.Replace(cmd, "\n", "\\n", -1))
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

func buildServiceCmd(service, args string) string {
	return fmt.Sprintf(`rosservice call %s "%s"`, service, args)
}

func buildTopicCmd(topic, msg, args string) string {
	onceFlag := ""
	if *pubOnce {
		onceFlag = "-1"
	}
	return fmt.Sprintf(`rostopic pub %s %s %s "%s"`, onceFlag, topic, msg, args)
}

func histAction(c *kingpin.ParseContext) error {
	if cfg.HistFile == "" {
		fmt.Println("histfile is not set.")
		return nil
	}
	if cfg.SelectCmd == "" {
		fmt.Println("selectcmd is not set.")
		return nil
	}
	cmd, err := selectCmdOutput("cat "+cfg.HistFile, cfg.SelectCmd)
	if err != nil {
		return err
	}
	cmd = strings.Replace(cmd, "\\n", "\n", -1)
	fmt.Println("--- IN ---")
	fmt.Println(cmd)
	fmt.Println("--- OUT ---")
	return execCommand(cmd)
}

func main() {
	app.Version(version)
	app.HelpFlag.Short('h')
	serviceCmd.Action(modeService.action())
	topicCmd.Action(modeTopic.action())
	histCmd.Action(histAction)

	cfg.load()

	kingpin.MustParse(app.Parse(os.Args[1:]))
}
