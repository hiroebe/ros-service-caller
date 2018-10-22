package main

import (
	"fmt"
	"io/ioutil"
	"log"
	"os"
	"os/exec"
	"strings"

	"github.com/urfave/cli"
)

func execCommand(command string) error {
	cmd := exec.Command("sh", "-c", command)
	cmd.Stderr = os.Stderr
	cmd.Stdout = os.Stdout
	cmd.Stdin = os.Stdin
	err := cmd.Run()
	return err
}

func execCommandOutput(command string) (string, error) {
	out, err := exec.Command("sh", "-c", command).Output()
	if err != nil {
		return "", err
	}
	return strings.TrimRight(string(out), "\n"), nil
}

func action(c *cli.Context) error {
	if c.NArg() == 0 {
		cli.ShowAppHelp(c)
		return nil
	}

	var err error
	service := c.Args().Get(0)

	if filename := c.String("file"); filename != "" {
		err = callServiceFromFile(service, filename)
		if err != nil {
			return err
		}
		return nil
	}

	filename := "tmp_ros_service_caller.yaml"

	proto, err := getSrvProto(service)
	if err != nil {
		return err
	}
	err = writeContent(filename, proto)
	if err != nil {
		return err
	}
	defer execCommand("rm " + filename)

	editor := os.Getenv("EDITOR")
	if editor == "" {
		editor = "vim"
	}
	err = execCommand(editor + " " + filename)
	if err != nil {
		return err
	}

	err = callServiceFromFile(service, filename)
	if err != nil {
		return err
	}

	return nil
}

func callServiceFromFile(service, file string) error {
	f, err := ioutil.ReadFile(file)
	if err != nil {
		return err
	}
	err = execCommand(fmt.Sprintf(`rosservice call %s "%s"`, service, string(f)))
	if err != nil {
		return err
	}
	return nil
}

func getSrvProto(service string) (string, error) {
	srv, err := execCommandOutput("rosservice type " + service)
	if err != nil {
		return "", err
	}
	content, err := execCommandOutput("rosmsg-proto srv " + srv)
	if err != nil {
		return "", err
	}
	content = strings.Trim(content, `"`)
	return content, nil
}

func writeContent(filename, content string) error {
	file, err := os.OpenFile(filename, os.O_WRONLY|os.O_CREATE, 0666)
	if err != nil {
		return err
	}
	defer file.Close()
	fmt.Fprint(file, content)
	return nil
}

func main() {
	app := cli.NewApp()
	app.Name = "ros-service-caller"
	app.Usage = "Edit args for rosservice-call with editor"
	app.UsageText = "ros-service-caller [global options] <service>"
	app.Version = "0.0.1"
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
