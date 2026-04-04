package ubx

import (
	"encoding/binary"
	"io"
)

// MaxFrameSize is the maximum allowed UBX frame size.
// UBX payload length is uint16 (max 65535) + 8 bytes overhead.
const MaxFrameSize = 65535 + HeaderLen

// Decoder reads UBX frames from an io.Reader.
// It implements a state machine that synchronizes on UBX sync bytes
// and yields complete, validated messages.
type Decoder struct {
	r   io.Reader
	buf []byte
	pos int // valid data in buf[0:pos]
}

// NewDecoder creates a new Decoder that reads from r.
func NewDecoder(r io.Reader) *Decoder {
	return &Decoder{
		r:   r,
		buf: make([]byte, 4096),
	}
}

// Decode reads the next UBX message from the underlying reader.
// It skips over non-UBX data (e.g., NMEA sentences) to find the next valid frame.
// Returns io.EOF when no more data is available.
func (d *Decoder) Decode() (Message, error) {
	for {
		msg, skip, _ := d.tryParse()
		if msg != nil {
			d.consume(skip)
			return msg, nil
		}

		// Skip bytes as indicated by tryParse
		if skip > 0 {
			d.consume(skip)
			continue
		}

		// Need more data (incomplete)
		if err := d.fill(); err != nil {
			return nil, err
		}
	}
}

// tryParse attempts to find and parse a UBX frame in the buffer.
// Returns (msg, consumed, nil) on success, or (nil, skipBytes, err) on failure.
func (d *Decoder) tryParse() (Message, int, error) {
	data := d.buf[:d.pos]

	// Scan for sync bytes
	for i := 0; i < len(data)-1; i++ {
		if data[i] != SyncByte1 || data[i+1] != SyncByte2 {
			continue
		}

		// Skip garbage bytes before sync
		if i > 0 {
			return nil, i, newParseError(ErrIncomplete, nil, "skipping %d garbage bytes", i)
		}

		// Check if we have enough bytes to read the length field
		if len(data) < 6 {
			return nil, 0, newParseError(ErrIncomplete, nil, "need length field")
		}

		// Validate declared frame size before attempting parse
		payloadLen := int(binary.LittleEndian.Uint16(data[4:6]))
		frameLen := payloadLen + HeaderLen
		if frameLen > MaxFrameSize {
			// Skip this sync pair entirely
			return nil, 2, newParseError(ErrPayloadLen, nil,
				"declared frame size %d exceeds maximum %d", frameLen, MaxFrameSize)
		}

		if len(data) < frameLen {
			return nil, 0, newParseError(ErrIncomplete, nil, "need %d bytes, have %d", frameLen, len(data))
		}

		msg, consumed, err := ParseFrame(data)
		if err != nil {
			// Checksum failure: skip past entire bad frame
			return nil, frameLen, err
		}
		return msg, consumed, nil
	}

	// No sync bytes found; discard all but the last byte (could be start of sync pair)
	if d.pos > 1 {
		return nil, d.pos - 1, newParseError(ErrIncomplete, nil, "no sync bytes found")
	}
	return nil, 0, newParseError(ErrIncomplete, nil, "insufficient data")
}

// fill reads more data from the underlying reader into the buffer.
func (d *Decoder) fill() error {
	// Cap buffer growth at MaxFrameSize + margin
	maxBuf := MaxFrameSize + 1024
	if d.pos == len(d.buf) {
		newSize := len(d.buf) * 2
		if newSize > maxBuf {
			newSize = maxBuf
		}
		if newSize <= len(d.buf) {
			// Buffer is at max and full; discard old data to make room
			half := d.pos / 2
			copy(d.buf, d.buf[half:d.pos])
			d.pos -= half
			return nil
		}
		newBuf := make([]byte, newSize)
		copy(newBuf, d.buf[:d.pos])
		d.buf = newBuf
	}

	n, err := d.r.Read(d.buf[d.pos:])
	d.pos += n
	if n > 0 {
		return nil
	}
	if err != nil {
		return err
	}
	return io.ErrNoProgress
}

// consume removes the first n bytes from the buffer.
func (d *Decoder) consume(n int) {
	if n <= 0 {
		return
	}
	if n >= d.pos {
		d.pos = 0
		return
	}
	copy(d.buf, d.buf[n:d.pos])
	d.pos -= n
}

func isIncomplete(err error) bool {
	if pe, ok := err.(*ParseError); ok {
		return pe.Kind == ErrIncomplete
	}
	return false
}
