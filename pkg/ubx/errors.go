package ubx

import (
	"errors"
	"fmt"
)

// ParseErrorKind identifies the category of a parse error.
type ParseErrorKind int

// Error kind constants.
const (
	ErrInvalidSync ParseErrorKind = iota + 1
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

// errIncomplete is a pre-allocated error for the common "need more data" case
// to avoid allocations on the hot path.
var errIncomplete = &ParseError{Kind: ErrIncomplete, Message: "incomplete frame"}

// ParseError represents a structured UBX parsing error.
type ParseError struct {
	Kind    ParseErrorKind // Error kind (ErrInvalidSync, ErrChecksum, etc.)
	Frame   []byte         // Raw frame bytes (may be nil for incomplete frames)
	Message string         // Human-readable error message
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

func newParseError(kind ParseErrorKind, frame []byte, format string, args ...any) *ParseError {
	// Copy frame bytes to avoid retaining references to mutable buffers.
	var frameCopy []byte
	if frame != nil {
		frameCopy = make([]byte, len(frame))
		copy(frameCopy, frame)
	}
	return &ParseError{
		Kind:    kind,
		Frame:   frameCopy,
		Message: fmt.Sprintf(format, args...),
	}
}
