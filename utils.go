package main

import (
	"bytes"
	"fmt"
	"io/ioutil"
	"os"
	"os/exec"
	"strings"
)

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

func selectCmdOutput(cmd, selectcmd string) (string, error) {
	f, err := ioutil.TempFile("", "")
	if err != nil {
		return "", err
	}
	defer os.Remove(f.Name())
	defer f.Close()

	err = execCommand(fmt.Sprintf("%s | %s > %s", cmd, selectcmd, f.Name()))
	if err != nil {
		return "", err
	}
	return readDataFromFile(f)
}

func appendToFile(filename, content string) error {
	if strings.HasPrefix(filename, "~") {
		filename = strings.Replace(filename, "~", os.Getenv("HOME"), 1)
	}

	file, err := os.OpenFile(filename, os.O_WRONLY|os.O_CREATE|os.O_APPEND, 0666)
	if err != nil {
		return err
	}
	defer file.Close()
	_, err = fmt.Fprintln(file, content)

	return err
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
