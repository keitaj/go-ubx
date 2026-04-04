//go:build darwin

package serialport

import (
	"fmt"
	"syscall"
)

const (
	ioctlGETATTR = syscall.TIOCGETA
	ioctlSETATTR = syscall.TIOCSETA
)

var baudRates = map[int]uint64{
	9600:   syscall.B9600,
	19200:  syscall.B19200,
	38400:  syscall.B38400,
	57600:  syscall.B57600,
	115200: syscall.B115200,
	230400: syscall.B230400,
}

func setBaudRate(t *syscall.Termios, baudRate int) error {
	speed, ok := baudRates[baudRate]
	if !ok {
		return fmt.Errorf("unsupported baud rate: %d", baudRate)
	}
	t.Ispeed = speed
	t.Ospeed = speed
	return nil
}
