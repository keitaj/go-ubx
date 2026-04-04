package ubx

import (
	"errors"
	"testing"
)

func TestEncodeFrame(t *testing.T) {
	// Encode a frame and verify it can be parsed back
	payload := []byte{0x01, 0x02, 0x03}
	frame := EncodeFrame(0xFF, 0xFE, payload)

	// Verify sync bytes
	if frame[0] != SyncByte1 || frame[1] != SyncByte2 {
		t.Errorf("sync: got 0x%02X 0x%02X, want 0xB5 0x62", frame[0], frame[1])
	}

	// Verify class + ID
	if frame[2] != 0xFF || frame[3] != 0xFE {
		t.Errorf("class/id: got 0x%02X 0x%02X, want 0xFF 0xFE", frame[2], frame[3])
	}

	// Verify length
	if frame[4] != 0x03 || frame[5] != 0x00 {
		t.Errorf("length: got 0x%02X 0x%02X, want 0x03 0x00", frame[4], frame[5])
	}

	// Verify it parses back successfully
	msg, frameLen, err := ParseFrame(frame)
	if err != nil {
		t.Fatalf("round-trip parse failed: %v", err)
	}
	if frameLen != len(frame) {
		t.Errorf("frameLen: got %d, want %d", frameLen, len(frame))
	}
	raw, ok := msg.(*RawMessage)
	if !ok {
		t.Fatalf("expected *RawMessage, got %T", msg)
	}
	if len(raw.Payload) != 3 {
		t.Errorf("payload len: got %d, want 3", len(raw.Payload))
	}
}

func TestEncodeFrameEmptyPayload(t *testing.T) {
	// Use an unknown class/ID so it falls through to RawMessage
	frame := EncodeFrame(0xFE, 0xFD, nil)
	if len(frame) != HeaderLen {
		t.Errorf("frame len: got %d, want %d", len(frame), HeaderLen)
	}

	msg, _, err := ParseFrame(frame)
	if err != nil {
		t.Fatalf("parse empty payload frame: %v", err)
	}
	raw, ok := msg.(*RawMessage)
	if !ok {
		t.Fatalf("expected *RawMessage, got %T", msg)
	}
	if len(raw.Payload) != 0 {
		t.Errorf("payload len: got %d, want 0", len(raw.Payload))
	}
}

func TestCfgValsetSingleKey(t *testing.T) {
	// Enable NAV-PVT on USB at rate 1
	frame := EnableMessageUSB(KeyMsgoutNavPvtUSB, 1)

	// Parse the frame
	msg, _, err := ParseFrame(frame)
	if err != nil {
		t.Fatalf("parse failed: %v", err)
	}

	// Should be a RawMessage with CFG-VALSET ClassID
	raw, ok := msg.(*RawMessage)
	if !ok {
		t.Fatalf("expected *RawMessage, got %T", msg)
	}
	if raw.GetClassID() != NewClassID(ClassCFG, IDCfgValset) {
		t.Errorf("classID: got %v, want CFG-VALSET", raw.GetClassID())
	}

	// Verify payload structure:
	// [version=0, layers=RAM, reserved, reserved, key(4), value(1)]
	p := raw.Payload
	if len(p) != 9 {
		t.Fatalf("payload len: got %d, want 9", len(p))
	}
	if p[0] != 0x00 {
		t.Errorf("version: got 0x%02X, want 0x00", p[0])
	}
	if p[1] != LayerRAM {
		t.Errorf("layers: got 0x%02X, want 0x%02X (RAM)", p[1], LayerRAM)
	}
	// Key at offset 4 (little-endian)
	key := uint32(p[4]) | uint32(p[5])<<8 | uint32(p[6])<<16 | uint32(p[7])<<24
	if key != KeyMsgoutNavPvtUSB {
		t.Errorf("key: got 0x%08X, want 0x%08X", key, KeyMsgoutNavPvtUSB)
	}
	if p[8] != 1 {
		t.Errorf("value: got %d, want 1", p[8])
	}
}

func TestCfgValsetMultipleKeys(t *testing.T) {
	frame := NewCfgValset(LayerRAM|LayerBBR).
		AddU1(KeyMsgoutNavPvtUSB, 1).
		AddU1(KeyMsgoutRxmRawxUSB, 1).
		AddU1(KeyMsgoutMonRfUSB, 1).
		Build()

	// Verify it parses without error
	msg, _, err := ParseFrame(frame)
	if err != nil {
		t.Fatalf("parse failed: %v", err)
	}

	raw := msg.(*RawMessage)
	// 4 (header) + 3 * (4 key + 1 value) = 19 bytes payload
	if len(raw.Payload) != 19 {
		t.Errorf("payload len: got %d, want 19", len(raw.Payload))
	}
	if raw.Payload[1] != LayerRAM|LayerBBR {
		t.Errorf("layers: got 0x%02X, want 0x%02X", raw.Payload[1], LayerRAM|LayerBBR)
	}
}

func TestCfgValsetU2(t *testing.T) {
	// Set measurement rate to 100ms (10Hz)
	frame := SetMeasurementRate(100)

	msg, _, err := ParseFrame(frame)
	if err != nil {
		t.Fatalf("parse failed: %v", err)
	}

	raw := msg.(*RawMessage)
	// 4 (header) + 4 (key) + 2 (U2 value) = 10 bytes
	if len(raw.Payload) != 10 {
		t.Errorf("payload len: got %d, want 10", len(raw.Payload))
	}
	// Value at offset 8 (little-endian U2)
	val := uint16(raw.Payload[8]) | uint16(raw.Payload[9])<<8
	if val != 100 {
		t.Errorf("rate: got %d, want 100", val)
	}
}

func TestDisableMessageUSB(t *testing.T) {
	frame := DisableMessageUSB(KeyMsgoutNavPvtUSB)

	msg, _, err := ParseFrame(frame)
	if err != nil {
		t.Fatalf("parse failed: %v", err)
	}

	raw := msg.(*RawMessage)
	// Value should be 0
	if raw.Payload[8] != 0 {
		t.Errorf("value: got %d, want 0", raw.Payload[8])
	}
}

func TestDecodeAckAck(t *testing.T) {
	// ACK-ACK for CFG-VALSET: class=0x06, id=0x8A
	frame := EncodeFrame(ClassACK, IDAckAck, []byte{ClassCFG, IDCfgValset})

	msg, _, err := ParseFrame(frame)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}

	ack, ok := msg.(*AckAck)
	if !ok {
		t.Fatalf("expected *AckAck, got %T", msg)
	}
	if ack.GetClassID() != NewClassID(ClassACK, IDAckAck) {
		t.Errorf("classID: got %v, want ACK-ACK", ack.GetClassID())
	}
	if ack.AckedClass != ClassCFG || ack.AckedID != IDCfgValset {
		t.Errorf("acked: got 0x%02X-0x%02X, want CFG-VALSET", ack.AckedClass, ack.AckedID)
	}
	if ack.AckedClassID() != NewClassID(ClassCFG, IDCfgValset) {
		t.Errorf("AckedClassID: got %v, want CFG-VALSET", ack.AckedClassID())
	}
}

func TestDecodeAckNak(t *testing.T) {
	// ACK-NAK for CFG-VALSET
	frame := EncodeFrame(ClassACK, IDAckNak, []byte{ClassCFG, IDCfgValset})

	msg, _, err := ParseFrame(frame)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}

	nak, ok := msg.(*AckNak)
	if !ok {
		t.Fatalf("expected *AckNak, got %T", msg)
	}
	if nak.GetClassID() != NewClassID(ClassACK, IDAckNak) {
		t.Errorf("classID: got %v, want ACK-NAK", nak.GetClassID())
	}
	if nak.NakedClassID() != NewClassID(ClassCFG, IDCfgValset) {
		t.Errorf("NakedClassID: got %v, want CFG-VALSET", nak.NakedClassID())
	}
}

func TestEncodeDecodeRoundTrip(t *testing.T) {
	// Build a CFG-VALSET, encode it, then ensure the checksum is valid
	frame := NewCfgValset(LayerRAM).
		AddU1(KeyMsgoutNavPvtUSB, 1).
		AddU1(KeyMsgoutRxmRawxUSB, 1).
		AddU1(KeyMsgoutRxmSfrbxUSB, 1).
		AddU1(KeyMsgoutMonRfUSB, 1).
		AddU2(KeyRateMeas, 1000).
		Build()

	// Corrupt the checksum
	bad := make([]byte, len(frame))
	copy(bad, frame)
	bad[len(bad)-1] ^= 0xFF

	_, _, err := ParseFrame(bad)
	if !errors.Is(err, ErrChecksumMismatch) {
		t.Errorf("corrupted frame should fail checksum, got: %v", err)
	}

	// Original should parse fine
	_, _, err = ParseFrame(frame)
	if err != nil {
		t.Fatalf("original frame should parse: %v", err)
	}
}
