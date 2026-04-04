//go:build darwin || linux

// Package serialport provides minimal serial port access using syscalls.
// No external dependencies — uses only the standard library syscall package.
package serialport

import (
	"fmt"
	"os"
	"syscall"
	"unsafe"
)

// Port wraps an os.File for serial communication.
type Port struct {
	file *os.File
}

// Open opens a serial port with the given baud rate.
// Configures 8N1 (8 data bits, no parity, 1 stop bit), raw mode.
func Open(name string, baudRate int) (*Port, error) {
	f, err := os.OpenFile(name, os.O_RDWR|syscall.O_NOCTTY, 0)
	if err != nil {
		return nil, fmt.Errorf("open %s: %w", name, err)
	}

	fd := int(f.Fd())

	// Get current terminal attributes
	var t syscall.Termios
	if _, _, errno := syscall.Syscall(
		syscall.SYS_IOCTL,
		uintptr(fd),
		ioctlGETATTR,
		uintptr(unsafe.Pointer(&t)),
	); errno != 0 {
		f.Close()
		return nil, fmt.Errorf("tcgetattr: %w", errno)
	}

	// Raw mode: no echo, no signals, no canonical processing
	t.Iflag &^= syscall.IGNBRK | syscall.BRKINT | syscall.PARMRK | syscall.ISTRIP |
		syscall.INLCR | syscall.IGNCR | syscall.ICRNL | syscall.IXON
	t.Oflag &^= syscall.OPOST
	t.Lflag &^= syscall.ECHO | syscall.ECHONL | syscall.ICANON | syscall.ISIG | syscall.IEXTEN
	t.Cflag &^= syscall.CSIZE | syscall.PARENB
	t.Cflag |= syscall.CS8 | syscall.CLOCAL | syscall.CREAD

	// Set baud rate (platform-specific)
	if err := setBaudRate(&t, baudRate); err != nil {
		f.Close()
		return nil, err
	}

	// Read: return after 1 byte or 1 second timeout
	t.Cc[syscall.VMIN] = 1
	t.Cc[syscall.VTIME] = 10 // 1 second (in tenths of a second)

	if _, _, errno := syscall.Syscall(
		syscall.SYS_IOCTL,
		uintptr(fd),
		ioctlSETATTR,
		uintptr(unsafe.Pointer(&t)),
	); errno != 0 {
		f.Close()
		return nil, fmt.Errorf("tcsetattr: %w", errno)
	}

	return &Port{file: f}, nil
}

func (p *Port) Read(buf []byte) (int, error)  { return p.file.Read(buf) }
func (p *Port) Write(buf []byte) (int, error) { return p.file.Write(buf) }
func (p *Port) Close() error                  { return p.file.Close() }
