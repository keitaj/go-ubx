// Package ubx provides a decoder and encoder for u-blox UBX binary protocol messages.
//
// Built-in message types: NAV-PVT, RXM-RAWX, MON-RF, RXM-SFRBX.
//
// Custom message types can be registered with [RegisterParser]:
//
//	ubx.RegisterParser(0x01, 0x03, func(classID ClassID, payload []byte) (ubx.Message, error) {
//	    // decode payload bytes
//	})
package ubx

import (
	"encoding/binary"
	"fmt"
	"math"
	"sync"
)

// ParserFunc is the function signature for custom message parsers.
type ParserFunc func(classID ClassID, payload []byte) (Message, error)

var (
	customParsers   = make(map[ClassID]ParserFunc)
	customParsersMu sync.RWMutex
)

// RegisterParser registers a custom parser for the given class and message ID.
// If a parser for the same ClassID is already registered, it is replaced.
// Built-in parsers can also be overridden.
func RegisterParser(class, id byte, parser ParserFunc) {
	customParsersMu.Lock()
	defer customParsersMu.Unlock()
	customParsers[NewClassID(class, id)] = parser
}

// UnregisterParser removes a custom parser for the given class and message ID.
func UnregisterParser(class, id byte) {
	customParsersMu.Lock()
	defer customParsersMu.Unlock()
	delete(customParsers, NewClassID(class, id))
}

// Sync bytes for UBX protocol.
const (
	SyncByte1 = 0xB5
	SyncByte2 = 0x62
)

// Message class constants.
const (
	ClassNAV byte = 0x01 // Navigation results
	ClassRXM byte = 0x02 // Receiver manager (raw data)
	ClassINF byte = 0x04 // Information messages
	ClassACK byte = 0x05 // Acknowledgment
	ClassCFG byte = 0x06 // Configuration
	ClassMON byte = 0x0A // Monitoring
	ClassTIM byte = 0x0D // Timing
	ClassSEC byte = 0x27 // Security
)

// Message ID constants.
const (
	IDNavPVT    byte = 0x07 // NAV-PVT
	IDRxmRAWX   byte = 0x15 // RXM-RAWX
	IDRxmSFRBX  byte = 0x13 // RXM-SFRBX
	IDMonRF     byte = 0x38 // MON-RF
	IDAckAck    byte = 0x01 // ACK-ACK
	IDAckNak    byte = 0x00 // ACK-NAK
	IDCfgValset byte = 0x8A // CFG-VALSET
	IDCfgValget byte = 0x8B // CFG-VALGET
)

// GNSS ID constants (used in UBX messages).
const (
	GnssIDGPS     byte = 0 // GPS
	GnssIDSBAS    byte = 1 // SBAS
	GnssIDGalileo byte = 2 // Galileo
	GnssIDBeiDou  byte = 3 // BeiDou
	GnssIDQZSS    byte = 5 // QZSS
	GnssIDGLONASS byte = 6 // GLONASS
)

// ClassID combines message class and ID into a single value.
type ClassID uint16

// NewClassID creates a ClassID from class and message ID bytes.
func NewClassID(class, id byte) ClassID {
	return ClassID(uint16(class)<<8 | uint16(id))
}

// Class returns the message class byte.
func (c ClassID) Class() byte { return byte(c >> 8) }

// ID returns the message ID byte.
func (c ClassID) ID() byte { return byte(c & 0xFF) }

func (c ClassID) String() string {
	names := map[ClassID]string{
		NewClassID(ClassNAV, IDNavPVT):    "NAV-PVT",
		NewClassID(ClassRXM, IDRxmRAWX):   "RXM-RAWX",
		NewClassID(ClassRXM, IDRxmSFRBX):  "RXM-SFRBX",
		NewClassID(ClassMON, IDMonRF):     "MON-RF",
		NewClassID(ClassACK, IDAckAck):    "ACK-ACK",
		NewClassID(ClassACK, IDAckNak):    "ACK-NAK",
		NewClassID(ClassCFG, IDCfgValset): "CFG-VALSET",
		NewClassID(ClassCFG, IDCfgValget): "CFG-VALGET",
	}
	if name, ok := names[c]; ok {
		return name
	}
	return fmt.Sprintf("0x%02X-0x%02X", c.Class(), c.ID())
}

// Message is the interface that all parsed UBX messages implement.
type Message interface {
	GetClassID() ClassID
}

// RawMessage represents an unknown or unregistered UBX message.
type RawMessage struct {
	MsgClassID ClassID
	Payload    []byte
}

func (m *RawMessage) GetClassID() ClassID { return m.MsgClassID }

// --- NAV-PVT (0x01 0x07) ---

// NavPVT represents a Navigation Position Velocity Time Solution message.
type NavPVT struct {
	MsgClassID ClassID
	ITOW       uint32 // GPS time of week (ms)
	Year       uint16 // Year (UTC)
	Month      uint8  // Month (1-12)
	Day        uint8  // Day (1-31)
	Hour       uint8  // Hour (0-23)
	Min        uint8  // Minute (0-59)
	Sec        uint8  // Second (0-60)
	Valid      uint8  // Validity flags
	TAcc       uint32 // Time accuracy estimate (ns)
	Nano       int32  // Fraction of second (-1e9..1e9 ns)
	FixType    uint8  // Fix type (0=No fix, 2=2D, 3=3D, 4=GNSS+DR, 5=Time only)
	Flags      uint8  // Fix status flags
	Flags2     uint8  // Additional flags
	NumSV      uint8  // Number of satellites used
	Lon        int32  // Longitude (1e-7 degrees)
	Lat        int32  // Latitude (1e-7 degrees)
	Height     int32  // Height above ellipsoid (mm)
	HMSL       int32  // Height above mean sea level (mm)
	HAcc       uint32 // Horizontal accuracy estimate (mm)
	VAcc       uint32 // Vertical accuracy estimate (mm)
	VelN       int32  // North velocity (mm/s)
	VelE       int32  // East velocity (mm/s)
	VelD       int32  // Down velocity (mm/s)
	GSpeed     int32  // Ground speed (mm/s)
	HeadMot    int32  // Heading of motion (1e-5 degrees)
	SAcc       uint32 // Speed accuracy estimate (mm/s)
	HeadAcc    uint32 // Heading accuracy estimate (1e-5 degrees)
	PDOP       uint16 // Position DOP (0.01)
	Flags3     uint16 // Additional flags
}

func (m *NavPVT) GetClassID() ClassID { return m.MsgClassID }

// LonDeg returns the longitude in decimal degrees.
func (m *NavPVT) LonDeg() float64 { return float64(m.Lon) * 1e-7 }

// LatDeg returns the latitude in decimal degrees.
func (m *NavPVT) LatDeg() float64 { return float64(m.Lat) * 1e-7 }

// HeightM returns the ellipsoid height in meters.
func (m *NavPVT) HeightM() float64 { return float64(m.Height) * 1e-3 }

// HMSLM returns the mean sea level height in meters.
func (m *NavPVT) HMSLM() float64 { return float64(m.HMSL) * 1e-3 }

// HAccM returns the horizontal accuracy in meters.
func (m *NavPVT) HAccM() float64 { return float64(m.HAcc) * 1e-3 }

// VAccM returns the vertical accuracy in meters.
func (m *NavPVT) VAccM() float64 { return float64(m.VAcc) * 1e-3 }

// PDOPVal returns the PDOP as a float64.
func (m *NavPVT) PDOPVal() float64 { return float64(m.PDOP) * 0.01 }

// --- RXM-RAWX (0x02 0x15) ---

// RxmRAWXMeas holds raw measurement data for a single satellite signal.
type RxmRAWXMeas struct {
	PrMes    float64 // Pseudorange measurement (m)
	CpMes    float64 // Carrier phase measurement (cycles)
	DoMes    float32 // Doppler measurement (Hz)
	GnssID   uint8   // GNSS identifier
	SvID     uint8   // Satellite identifier
	SigID    uint8   // Signal identifier
	FreqID   uint8   // GLONASS frequency slot (0-13)
	Locktime uint16  // Carrier phase locktime counter (ms)
	CNO      uint8   // Carrier-to-noise ratio (dB-Hz)
	PrStdev  uint8   // Pseudorange std dev (0.01*2^n m, 4-bit)
	CpStdev  uint8   // Carrier phase std dev (0.004 cycles, 4-bit)
	DoStdev  uint8   // Doppler std dev (0.002*2^n Hz, 4-bit)
	TrkStat  uint8   // Tracking status bitfield
}

// PrValid returns true if the pseudorange measurement is valid.
func (m *RxmRAWXMeas) PrValid() bool { return m.TrkStat&0x01 != 0 }

// CpValid returns true if the carrier phase measurement is valid.
func (m *RxmRAWXMeas) CpValid() bool { return m.TrkStat&0x02 != 0 }

// HalfCyc returns true if half cycle ambiguity is present.
func (m *RxmRAWXMeas) HalfCyc() bool { return m.TrkStat&0x04 != 0 }

// RxmRAWX represents a Multi-GNSS Raw Measurements message.
type RxmRAWX struct {
	MsgClassID ClassID
	RcvTow     float64       // Measurement time of week (s)
	Week       uint16        // GPS week number
	LeapS      int8          // GPS-UTC leap seconds
	NumMeas    uint8         // Number of measurements
	RecStat    uint8         // Receiver status flags
	Version    uint8         // Message version
	Meas       []RxmRAWXMeas // Per-satellite measurements
}

func (m *RxmRAWX) GetClassID() ClassID { return m.MsgClassID }

// --- MON-RF (0x0A 0x38) ---

// MonRFBlock holds RF information for a single RF block.
type MonRFBlock struct {
	BlockID    uint8  // RF block ID
	Flags      uint8  // Jamming state (0=unknown, 1=OK, 2=warning, 3=critical)
	AntStatus  uint8  // Antenna supervisor state
	AntPower   uint8  // Antenna power status
	PostStatus uint32 // POST status word
	NoisePerMS uint16 // Noise level
	AgcCnt     uint16 // AGC monitor (0-8191)
	JamInd     uint8  // CW jamming indicator (0-255)
	OfsI       int8   // Imbalance of I-part
	MagI       uint8  // Magnitude of I-part
	OfsQ       int8   // Imbalance of Q-part
	MagQ       uint8  // Magnitude of Q-part
}

// JammingState returns a human-readable jamming state.
func (b *MonRFBlock) JammingState() string {
	states := []string{"unknown", "OK", "warning", "critical"}
	s := b.Flags & 0x03
	if int(s) < len(states) {
		return states[s]
	}
	return "unknown"
}

// MonRF represents an RF Information message.
type MonRF struct {
	MsgClassID ClassID
	Version    uint8        // Message version
	NBlocks    uint8        // Number of RF blocks
	Blocks     []MonRFBlock // Per-block RF information
}

func (m *MonRF) GetClassID() ClassID { return m.MsgClassID }

// --- RXM-SFRBX (0x02 0x13) ---

// RxmSFRBX represents a Raw Subframe Data message.
type RxmSFRBX struct {
	MsgClassID ClassID
	GnssID     uint8    // GNSS identifier
	SvID       uint8    // Satellite identifier
	FreqID     uint8    // GLONASS frequency slot
	NumWords   uint8    // Number of data words
	Channel    uint8    // Tracking channel number
	Version    uint8    // Message version
	Dwrd       []uint32 // Navigation data words
}

func (m *RxmSFRBX) GetClassID() ClassID { return m.MsgClassID }

// --- ACK-ACK / ACK-NAK (0x05 0x01 / 0x05 0x00) ---

// AckAck represents an acknowledgment of a CFG message.
type AckAck struct {
	MsgClassID ClassID
	AckedClass byte // Class of the acknowledged message
	AckedID    byte // ID of the acknowledged message
}

func (m *AckAck) GetClassID() ClassID { return m.MsgClassID }

// AckedClassID returns the ClassID of the message that was acknowledged.
func (m *AckAck) AckedClassID() ClassID { return NewClassID(m.AckedClass, m.AckedID) }

// AckNak represents a negative acknowledgment of a CFG message.
type AckNak struct {
	MsgClassID ClassID
	NakedClass byte // Class of the rejected message
	NakedID    byte // ID of the rejected message
}

func (m *AckNak) GetClassID() ClassID { return m.MsgClassID }

// NakedClassID returns the ClassID of the message that was rejected.
func (m *AckNak) NakedClassID() ClassID { return NewClassID(m.NakedClass, m.NakedID) }

// --- CFG-VALSET / CFG-VALGET (0x06 0x8A / 0x06 0x8B) ---

// Configuration layer constants for CFG-VALSET.
const (
	LayerRAM   byte = 0x01 // Volatile memory (lost on reset)
	LayerBBR   byte = 0x02 // Battery-backed RAM (survives reset)
	LayerFlash byte = 0x04 // Flash memory (permanent)
)

// CfgKeyVal represents a single configuration key-value pair.
type CfgKeyVal struct {
	Key uint32 // Configuration key ID
	Val []byte // Value bytes (size depends on key)
}

// --- CFG-VALGET (0x06 0x8B) ---

// CfgValget represents a CFG-VALGET response from the receiver.
type CfgValget struct {
	MsgClassID ClassID
	Version    uint8       // Message version
	Layer      uint8       // Layer from which values were read
	KeyVals    []CfgKeyVal // Configuration key-value pairs
}

func (m *CfgValget) GetClassID() ClassID { return m.MsgClassID }

// GetU1 returns the uint8 value for the given key, or 0 if not found.
func (m *CfgValget) GetU1(key uint32) (uint8, bool) {
	for _, kv := range m.KeyVals {
		if kv.Key == key && len(kv.Val) >= 1 {
			return kv.Val[0], true
		}
	}
	return 0, false
}

// GetU2 returns the uint16 value for the given key, or 0 if not found.
func (m *CfgValget) GetU2(key uint32) (uint16, bool) {
	for _, kv := range m.KeyVals {
		if kv.Key == key && len(kv.Val) >= 2 {
			return binary.LittleEndian.Uint16(kv.Val), true
		}
	}
	return 0, false
}

// GetU4 returns the uint32 value for the given key, or 0 if not found.
func (m *CfgValget) GetU4(key uint32) (uint32, bool) {
	for _, kv := range m.KeyVals {
		if kv.Key == key && len(kv.Val) >= 4 {
			return binary.LittleEndian.Uint32(kv.Val), true
		}
	}
	return 0, false
}

// --- Frame parsing ---

// HeaderLen is the total overhead: sync(2) + class(1) + id(1) + length(2) + checksum(2) = 8.
const HeaderLen = 8

// Checksum computes the UBX Fletcher-8 checksum over the given data.
// The data should include class, ID, length, and payload (not sync bytes).
func Checksum(data []byte) (ckA, ckB byte) {
	for _, b := range data {
		ckA += b
		ckB += ckA
	}
	return ckA, ckB
}

// ParseFrame parses a single UBX frame from raw bytes.
// The input must start with the sync bytes (0xB5 0x62) and contain a complete frame.
// Returns the parsed message, total frame length consumed, and any error.
func ParseFrame(data []byte) (Message, int, error) {
	if len(data) < HeaderLen {
		return nil, 0, newParseError(ErrIncomplete, nil, "need at least %d bytes, got %d", HeaderLen, len(data))
	}
	if data[0] != SyncByte1 || data[1] != SyncByte2 {
		return nil, 0, newParseError(ErrInvalidSync, data, "invalid sync bytes: 0x%02X 0x%02X", data[0], data[1])
	}

	class := data[2]
	id := data[3]
	payloadLen := int(binary.LittleEndian.Uint16(data[4:6]))
	frameLen := payloadLen + HeaderLen

	if len(data) < frameLen {
		return nil, 0, newParseError(ErrIncomplete, nil, "frame incomplete: need %d bytes, got %d", frameLen, len(data))
	}

	// Verify checksum over class + id + length + payload
	ckA, ckB := Checksum(data[2 : frameLen-2])
	if ckA != data[frameLen-2] || ckB != data[frameLen-1] {
		return nil, frameLen, newParseError(ErrChecksum, data[:frameLen],
			"checksum mismatch: expected 0x%02X 0x%02X, got 0x%02X 0x%02X",
			data[frameLen-2], data[frameLen-1], ckA, ckB)
	}

	payload := data[6 : 6+payloadLen]
	classID := NewClassID(class, id)

	// Check custom parsers first
	customParsersMu.RLock()
	parser, ok := customParsers[classID]
	customParsersMu.RUnlock()
	if ok {
		msg, err := parser(classID, payload)
		return msg, frameLen, err
	}

	msg, err := decodeMessage(classID, payload)
	return msg, frameLen, err
}

// decodeMessage dispatches to the appropriate built-in decoder.
func decodeMessage(classID ClassID, payload []byte) (Message, error) {
	switch classID {
	case NewClassID(ClassNAV, IDNavPVT):
		return decodeNavPVT(classID, payload)
	case NewClassID(ClassRXM, IDRxmRAWX):
		return decodeRxmRAWX(classID, payload)
	case NewClassID(ClassMON, IDMonRF):
		return decodeMonRF(classID, payload)
	case NewClassID(ClassRXM, IDRxmSFRBX):
		return decodeRxmSFRBX(classID, payload)
	case NewClassID(ClassACK, IDAckAck):
		return decodeAck(classID, payload)
	case NewClassID(ClassACK, IDAckNak):
		return decodeNak(classID, payload)
	case NewClassID(ClassCFG, IDCfgValget):
		return decodeCfgValget(classID, payload)
	default:
		return &RawMessage{MsgClassID: classID, Payload: append([]byte(nil), payload...)}, nil
	}
}

// --- Message decoders ---

func decodeNavPVT(classID ClassID, p []byte) (*NavPVT, error) {
	if len(p) < 84 {
		return nil, newParseError(ErrPayloadLen, p, "NAV-PVT requires at least 84 bytes, got %d", len(p))
	}
	m := &NavPVT{MsgClassID: classID}
	m.ITOW = binary.LittleEndian.Uint32(p[0:4])
	m.Year = binary.LittleEndian.Uint16(p[4:6])
	m.Month = p[6]
	m.Day = p[7]
	m.Hour = p[8]
	m.Min = p[9]
	m.Sec = p[10]
	m.Valid = p[11]
	m.TAcc = binary.LittleEndian.Uint32(p[12:16])
	m.Nano = int32(binary.LittleEndian.Uint32(p[16:20]))
	m.FixType = p[20]
	m.Flags = p[21]
	m.Flags2 = p[22]
	m.NumSV = p[23]
	m.Lon = int32(binary.LittleEndian.Uint32(p[24:28]))
	m.Lat = int32(binary.LittleEndian.Uint32(p[28:32]))
	m.Height = int32(binary.LittleEndian.Uint32(p[32:36]))
	m.HMSL = int32(binary.LittleEndian.Uint32(p[36:40]))
	m.HAcc = binary.LittleEndian.Uint32(p[40:44])
	m.VAcc = binary.LittleEndian.Uint32(p[44:48])
	m.VelN = int32(binary.LittleEndian.Uint32(p[48:52]))
	m.VelE = int32(binary.LittleEndian.Uint32(p[52:56]))
	m.VelD = int32(binary.LittleEndian.Uint32(p[56:60]))
	m.GSpeed = int32(binary.LittleEndian.Uint32(p[60:64]))
	m.HeadMot = int32(binary.LittleEndian.Uint32(p[64:68]))
	m.SAcc = binary.LittleEndian.Uint32(p[68:72])
	m.HeadAcc = binary.LittleEndian.Uint32(p[72:76])
	m.PDOP = binary.LittleEndian.Uint16(p[76:78])
	m.Flags3 = binary.LittleEndian.Uint16(p[78:80])
	return m, nil
}

func decodeRxmRAWX(classID ClassID, p []byte) (*RxmRAWX, error) {
	const headerSize = 16
	const measSize = 32
	if len(p) < headerSize {
		return nil, newParseError(ErrPayloadLen, p, "RXM-RAWX requires at least %d bytes, got %d", headerSize, len(p))
	}

	m := &RxmRAWX{MsgClassID: classID}
	m.RcvTow = float64FromBytes(p[0:8])
	m.Week = binary.LittleEndian.Uint16(p[8:10])
	m.LeapS = int8(p[10])
	m.NumMeas = p[11]
	m.RecStat = p[12]
	m.Version = p[13]

	expected := headerSize + int(m.NumMeas)*measSize
	if len(p) < expected {
		return nil, newParseError(ErrPayloadLen, p, "RXM-RAWX: need %d bytes for %d measurements, got %d",
			expected, m.NumMeas, len(p))
	}

	m.Meas = make([]RxmRAWXMeas, m.NumMeas)
	for i := 0; i < int(m.NumMeas); i++ {
		off := headerSize + i*measSize
		d := p[off : off+measSize]
		m.Meas[i] = RxmRAWXMeas{
			PrMes:    float64FromBytes(d[0:8]),
			CpMes:    float64FromBytes(d[8:16]),
			DoMes:    float32FromBytes(d[16:20]),
			GnssID:   d[20],
			SvID:     d[21],
			SigID:    d[22],
			FreqID:   d[23],
			Locktime: binary.LittleEndian.Uint16(d[24:26]),
			CNO:      d[26],
			PrStdev:  d[27] & 0x0F,
			CpStdev:  d[28] & 0x0F,
			DoStdev:  d[29] & 0x0F,
			TrkStat:  d[30],
		}
	}

	return m, nil
}

func decodeMonRF(classID ClassID, p []byte) (*MonRF, error) {
	const headerSize = 4
	const blockSize = 24
	if len(p) < headerSize {
		return nil, newParseError(ErrPayloadLen, p, "MON-RF requires at least %d bytes, got %d", headerSize, len(p))
	}

	m := &MonRF{MsgClassID: classID}
	m.Version = p[0]
	m.NBlocks = p[1]

	expected := headerSize + int(m.NBlocks)*blockSize
	if len(p) < expected {
		return nil, newParseError(ErrPayloadLen, p, "MON-RF: need %d bytes for %d blocks, got %d",
			expected, m.NBlocks, len(p))
	}

	m.Blocks = make([]MonRFBlock, m.NBlocks)
	for i := 0; i < int(m.NBlocks); i++ {
		off := headerSize + i*blockSize
		d := p[off : off+blockSize]
		m.Blocks[i] = MonRFBlock{
			BlockID:    d[0],
			Flags:      d[1],
			AntStatus:  d[2],
			AntPower:   d[3],
			PostStatus: binary.LittleEndian.Uint32(d[4:8]),
			NoisePerMS: binary.LittleEndian.Uint16(d[8:10]),
			AgcCnt:     binary.LittleEndian.Uint16(d[12:14]),
			JamInd:     d[16],
			OfsI:       int8(d[17]),
			MagI:       d[18],
			OfsQ:       int8(d[19]),
			MagQ:       d[20],
		}
	}

	return m, nil
}

func decodeRxmSFRBX(classID ClassID, p []byte) (*RxmSFRBX, error) {
	const headerSize = 8
	if len(p) < headerSize {
		return nil, newParseError(ErrPayloadLen, p, "RXM-SFRBX requires at least %d bytes, got %d", headerSize, len(p))
	}

	m := &RxmSFRBX{MsgClassID: classID}
	m.GnssID = p[0]
	m.SvID = p[1]
	m.FreqID = p[3]
	m.NumWords = p[4]
	m.Channel = p[5]
	m.Version = p[6]

	expected := headerSize + int(m.NumWords)*4
	if len(p) < expected {
		return nil, newParseError(ErrPayloadLen, p, "RXM-SFRBX: need %d bytes for %d words, got %d",
			expected, m.NumWords, len(p))
	}

	m.Dwrd = make([]uint32, m.NumWords)
	for i := 0; i < int(m.NumWords); i++ {
		off := headerSize + i*4
		m.Dwrd[i] = binary.LittleEndian.Uint32(p[off : off+4])
	}

	return m, nil
}

func decodeCfgValget(classID ClassID, p []byte) (*CfgValget, error) {
	if len(p) < 4 {
		return nil, newParseError(ErrPayloadLen, p, "CFG-VALGET requires at least 4 bytes, got %d", len(p))
	}

	m := &CfgValget{MsgClassID: classID}
	m.Version = p[0]
	m.Layer = p[1]

	// Parse key-value pairs from offset 4 onwards.
	// Value size is determined by the key's top byte (size/type indicator).
	off := 4
	for off+4 <= len(p) {
		key := binary.LittleEndian.Uint32(p[off : off+4])
		off += 4

		valSize := cfgKeyValueSize(key)
		if off+valSize > len(p) {
			break
		}

		val := make([]byte, valSize)
		copy(val, p[off:off+valSize])
		m.KeyVals = append(m.KeyVals, CfgKeyVal{Key: key, Val: val})
		off += valSize
	}

	return m, nil
}

// cfgKeyValueSize returns the value size in bytes based on the key's type indicator.
// The top byte of a configuration key encodes the storage size:
//
//	0x10 = 1 bit (L, stored as 1 byte)
//	0x20 = 1 byte (U1/E1/X1)
//	0x30 = 2 bytes (U2/E2/X2)
//	0x40 = 4 bytes (U4/I4/E4/X4/R4)
//	0x50 = 8 bytes (U8/R8)
func cfgKeyValueSize(key uint32) int {
	switch key >> 24 {
	case 0x10, 0x20:
		return 1
	case 0x30:
		return 2
	case 0x40:
		return 4
	case 0x50:
		return 8
	default:
		return 1 // fallback
	}
}

func decodeAck(classID ClassID, p []byte) (*AckAck, error) {
	if len(p) < 2 {
		return nil, newParseError(ErrPayloadLen, p, "ACK-ACK requires 2 bytes, got %d", len(p))
	}
	return &AckAck{MsgClassID: classID, AckedClass: p[0], AckedID: p[1]}, nil
}

func decodeNak(classID ClassID, p []byte) (*AckNak, error) {
	if len(p) < 2 {
		return nil, newParseError(ErrPayloadLen, p, "ACK-NAK requires 2 bytes, got %d", len(p))
	}
	return &AckNak{MsgClassID: classID, NakedClass: p[0], NakedID: p[1]}, nil
}

// --- Binary helpers ---

func float64FromBytes(b []byte) float64 {
	bits := binary.LittleEndian.Uint64(b)
	return math.Float64frombits(bits)
}

func float32FromBytes(b []byte) float32 {
	bits := binary.LittleEndian.Uint32(b)
	return math.Float32frombits(bits)
}
