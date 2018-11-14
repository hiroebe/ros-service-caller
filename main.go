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
)

func serviceAction(c *kingpin.ParseContext) error {
	var err error

	if err = checkRoscoreRunning(); err != nil {
		return err
	}

	if *service == "" {
		if cfg.SelectCmd == "" {
			fmt.Println("selectcmd is not set.")
			return nil
		}
		*service, err = selectNode("service", cfg.SelectCmd)
		if err != nil {
			return err
		}
	}

	var args string

	if *datafile != nil {
		args, err = readDataFromFile(*datafile)
		if err != nil {
			return err
		}
	} else {
		srv, err := execCommandOutput("rosservice type " + *service)
		if err != nil {
			return err
		}
		proto, err := getProto("srv", string(srv))
		if err != nil {
			return err
		}
		args, err = editArgs(proto)
		if err != nil {
			return err
		}
	}

	return callService(*service, args)
}

func topicAction(c *kingpin.ParseContext) error {
	var err error

	if err = checkRoscoreRunning(); err != nil {
		return err
	}

	if *topic == "" {
		if cfg.SelectCmd == "" {
			fmt.Println("selectcmd is not set.")
			return nil
		}
		*topic, err = selectNode("topic", cfg.SelectCmd)
		if err != nil {
			return err
		}
	}

	m, err := execCommandOutput("rostopic type " + *topic)
	if err != nil {
		return err
	}
	msg := string(m)

	var args string

	if *datafile != nil {
		args, err = readDataFromFile(*datafile)
		if err != nil {
			return err
		}
	} else {
		proto, err := getProto("msg", msg)
		if err != nil {
			return err
		}
		args, err = editArgs(proto)
		if err != nil {
			return err
		}
	}

	return pubTopic(*topic, msg, args)
}

func checkRoscoreRunning() error {
	if err := execCommand("rosnode list 1>/dev/null"); err != nil {
		return err
	}
	return nil
}

func selectNode(mode, selectcmd string) (string, error) {
	f, err := ioutil.TempFile("", "")
	if err != nil {
		return "", err
	}
	defer os.Remove(f.Name())
	defer f.Close()

	err = execCommand(fmt.Sprintf("ros%s list | %s > %s", mode, selectcmd, f.Name()))
	if err != nil {
		return "", err
	}
	return readDataFromFile(f)
}

func getProto(mode, target string) ([]byte, error) {
	content, err := execCommandOutput(fmt.Sprintf("rosmsg-proto %s %s", mode, target))
	if err != nil {
		return nil, err
	}
	content = bytes.Trim(content, `"`)
	return content, nil
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

func callService(s, a string) error {
	cmd := fmt.Sprintf(`rosservice call %s "%s"`, s, a)
	fmt.Println("--- IN ---")
	fmt.Println(cmd)
	fmt.Println("--- OUT ---")
	return execCommand(cmd)
}

func pubTopic(t, m, a string) error {
	cmd := fmt.Sprintf(`rostopic pub %s %s "%s"`, t, m, a)
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
	serviceCmd.Action(serviceAction)
	topicCmd.Action(topicAction)

	cfg.load()

	kingpin.MustParse(app.Parse(os.Args[1:]))
}
