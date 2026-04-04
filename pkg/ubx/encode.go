package ubx

import (
	"encoding/binary"
	"fmt"
)

// EncodeFrame builds a complete UBX frame from a class, ID, and payload.
// Returns the frame bytes including sync header and checksum.
// Panics if the payload exceeds the maximum UBX payload size (65535 bytes).
func EncodeFrame(class, id byte, payload []byte) []byte {
	payloadLen := len(payload)
	if payloadLen > 65535 {
		panic(fmt.Sprintf("ubx: payload size %d exceeds maximum 65535", payloadLen))
	}
	frame := make([]byte, payloadLen+HeaderLen)

	// Sync bytes
	frame[0] = SyncByte1
	frame[1] = SyncByte2

	// Class + ID
	frame[2] = class
	frame[3] = id

	// Payload length (little-endian)
	binary.LittleEndian.PutUint16(frame[4:6], uint16(payloadLen))

	// Payload
	copy(frame[6:], payload)

	// Checksum over class + id + length + payload
	ckA, ckB := Checksum(frame[2 : 6+payloadLen])
	frame[6+payloadLen] = ckA
	frame[6+payloadLen+1] = ckB

	return frame
}

// --- CFG-VALSET encoding ---

// Common configuration key IDs for u-blox F9P.
// Format: 0xSSKKKKKK where SS = size/type, KKKKKK = key ID.
const (
	// Message output rate keys (USB port)
	KeyMsgoutNavPvtUSB   uint32 = 0x20910009 // CFG-MSGOUT-UBX_NAV_PVT_USB
	KeyMsgoutRxmRawxUSB  uint32 = 0x209102A7 // CFG-MSGOUT-UBX_RXM_RAWX_USB
	KeyMsgoutRxmSfrbxUSB uint32 = 0x20910234 // CFG-MSGOUT-UBX_RXM_SFRBX_USB
	KeyMsgoutMonRfUSB    uint32 = 0x20910362 // CFG-MSGOUT-UBX_MON_RF_USB

	// Measurement rate
	KeyRateMeas uint32 = 0x30210001 // CFG-RATE-MEAS (ms, U2)
	KeyRateNav  uint32 = 0x30210002 // CFG-RATE-NAV (cycles, U2)
)

// CfgValsetBuilder builds a CFG-VALSET message payload.
type CfgValsetBuilder struct {
	layers byte
	kvs    []CfgKeyVal
}

// NewCfgValset creates a new CFG-VALSET builder targeting the given layers.
// Use LayerRAM, LayerBBR, LayerFlash constants (can be OR'd together).
func NewCfgValset(layers byte) *CfgValsetBuilder {
	return &CfgValsetBuilder{layers: layers}
}

// AddU1 adds a uint8 configuration value.
func (b *CfgValsetBuilder) AddU1(key uint32, val uint8) *CfgValsetBuilder {
	b.kvs = append(b.kvs, CfgKeyVal{Key: key, Val: []byte{val}})
	return b
}

// AddU2 adds a uint16 configuration value.
func (b *CfgValsetBuilder) AddU2(key uint32, val uint16) *CfgValsetBuilder {
	buf := make([]byte, 2)
	binary.LittleEndian.PutUint16(buf, val)
	b.kvs = append(b.kvs, CfgKeyVal{Key: key, Val: buf})
	return b
}

// AddU4 adds a uint32 configuration value.
func (b *CfgValsetBuilder) AddU4(key uint32, val uint32) *CfgValsetBuilder {
	buf := make([]byte, 4)
	binary.LittleEndian.PutUint32(buf, val)
	b.kvs = append(b.kvs, CfgKeyVal{Key: key, Val: buf})
	return b
}

// Build returns the complete UBX frame bytes for this CFG-VALSET.
func (b *CfgValsetBuilder) Build() []byte {
	// Payload: version(1) + layers(1) + reserved(2) + key-value pairs
	size := 4 // header
	for _, kv := range b.kvs {
		size += 4 + len(kv.Val) // key(4) + value
	}

	payload := make([]byte, size)
	payload[0] = 0x00 // version
	payload[1] = b.layers
	// payload[2:4] reserved = 0

	off := 4
	for _, kv := range b.kvs {
		binary.LittleEndian.PutUint32(payload[off:off+4], kv.Key)
		off += 4
		copy(payload[off:], kv.Val)
		off += len(kv.Val)
	}

	return EncodeFrame(ClassCFG, IDCfgValset, payload)
}

// --- CFG-VALGET encoding (poll request) ---

// PollCfgValget returns a UBX frame that polls the receiver for configuration values.
// layer specifies which layer to read from (0=RAM, 1=BBR, 2=Flash, 7=Default).
// keys are the configuration key IDs to query.
func PollCfgValget(layer byte, keys ...uint32) []byte {
	// Payload: version(1) + layer(1) + position(2) + keys(4 each)
	payload := make([]byte, 4+len(keys)*4)
	payload[0] = 0x00 // version
	payload[1] = layer
	// payload[2:4] = position (0 for single request)

	for i, key := range keys {
		binary.LittleEndian.PutUint32(payload[4+i*4:], key)
	}

	return EncodeFrame(ClassCFG, IDCfgValget, payload)
}

// CFG-VALGET layer constants for polling.
const (
	PollLayerRAM     byte = 0 // Read from RAM (current active values)
	PollLayerBBR     byte = 1 // Read from BBR
	PollLayerFlash   byte = 2 // Read from Flash
	PollLayerDefault byte = 7 // Read factory defaults
)

// --- Convenience functions ---

// EnableMessageUSB returns a UBX frame that enables a message type on the USB port.
// Rate is the output rate in navigation solution cycles (1 = every epoch).
func EnableMessageUSB(msgKey uint32, rate uint8) []byte {
	return NewCfgValset(LayerRAM).AddU1(msgKey, rate).Build()
}

// DisableMessageUSB returns a UBX frame that disables a message type on the USB port.
func DisableMessageUSB(msgKey uint32) []byte {
	return NewCfgValset(LayerRAM).AddU1(msgKey, 0).Build()
}

// SetMeasurementRate returns a UBX frame that sets the measurement rate.
// intervalMs is the measurement interval in milliseconds (e.g., 1000 = 1Hz, 100 = 10Hz).
func SetMeasurementRate(intervalMs uint16) []byte {
	return NewCfgValset(LayerRAM).AddU2(KeyRateMeas, intervalMs).Build()
}
