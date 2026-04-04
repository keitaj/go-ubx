package ubx

import (
	"errors"
	"fmt"
)

// Error kind constants.
const (
	ErrInvalidSync = iota + 1
	ErrChecksum
	ErrPayloadLen
	ErrIncomplete
)

// Sentinel errors for use with errors.Is.
var (
	ErrInvalidSyncBytes = errors.New("ubx: invalid sync bytes")
	ErrChecksumMismatch = errors.New("ubx: checksum mismatch")
	ErrInvalidPayload   = errors.New("ubx: invalid payload length")
	ErrFrameIncomplete  = errors.New("ubx: frame incomplete")
)

// ParseError represents a structured UBX parsing error.
type ParseError struct {
	Kind    int    // Error kind (ErrInvalidSync, ErrChecksum, etc.)
	Frame   []byte // Raw frame bytes (may be nil for incomplete frames)
	Message string // Human-readable error message
}

func (e *ParseError) Error() string {
	return e.Message
}

func (e *ParseError) Unwrap() error {
	switch e.Kind {
	case ErrInvalidSync:
		return ErrInvalidSyncBytes
	case ErrChecksum:
		return ErrChecksumMismatch
	case ErrPayloadLen:
		return ErrInvalidPayload
	case ErrIncomplete:
		return ErrFrameIncomplete
	default:
		return nil
	}
}

func newParseError(kind int, frame []byte, format string, args ...any) *ParseError {
	return &ParseError{
		Kind:    kind,
		Frame:   frame,
		Message: fmt.Sprintf(format, args...),
	}
}
