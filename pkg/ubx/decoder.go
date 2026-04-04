package ubx

import (
	"io"
)

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
		// Try to find and parse a frame from buffered data
		msg, consumed, err := d.tryParse()
		if msg != nil {
			d.consume(consumed)
			return msg, nil
		}
		if err != nil && !isIncomplete(err) {
			// Skip bad byte and retry
			if d.pos > 0 {
				d.consume(1)
				continue
			}
		}

		// Need more data
		if err := d.fill(); err != nil {
			return nil, err
		}
	}
}

// tryParse attempts to find and parse a UBX frame in the buffer.
func (d *Decoder) tryParse() (Message, int, error) {
	data := d.buf[:d.pos]

	// Scan for sync bytes
	for i := 0; i < len(data)-1; i++ {
		if data[i] != SyncByte1 || data[i+1] != SyncByte2 {
			continue
		}

		// Skip any garbage bytes before sync
		if i > 0 {
			d.consume(i)
			data = d.buf[:d.pos]
			i = 0
		}

		msg, frameLen, err := ParseFrame(data)
		if err != nil {
			return nil, 0, err
		}
		return msg, frameLen, nil
	}

	return nil, 0, newParseError(ErrIncomplete, nil, "no sync bytes found")
}

// fill reads more data from the underlying reader into the buffer.
func (d *Decoder) fill() error {
	// Grow buffer if needed
	if d.pos == len(d.buf) {
		newBuf := make([]byte, len(d.buf)*2)
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
