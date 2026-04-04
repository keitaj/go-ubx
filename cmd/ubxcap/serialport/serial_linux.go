//go:build linux

package serialport

import (
	"fmt"
	"syscall"
)

const (
	ioctlGETATTR = syscall.TCGETS
	ioctlSETATTR = syscall.TCSETS
)

// cbaud is the baud rate mask for Cflag.
// Not exported by the syscall package, but defined in asm-generic/termbits.h.
const cbaud = 0x100f

var baudRates = map[int]uint32{
	9600:   syscall.B9600,
	19200:  syscall.B19200,
	38400:  syscall.B38400,
	57600:  syscall.B57600,
	115200: syscall.B115200,
	230400: syscall.B230400,
	460800: syscall.B460800,
	921600: syscall.B921600,
}

func setBaudRate(t *syscall.Termios, baudRate int) error {
	speed, ok := baudRates[baudRate]
	if !ok {
		return fmt.Errorf("unsupported baud rate: %d", baudRate)
	}
	t.Cflag &^= cbaud
	t.Cflag |= speed
	return nil
}
