package ubx

import (
	"bytes"
	"errors"
	"io"
	"math"
	"testing"
)

const tolerance = 0.0001

func almostEqual(a, b float64) bool {
	return math.Abs(a-b) < tolerance
}

// Test frames generated with correct Fletcher-8 checksums.
var navPVTFrame = []byte{
	0xB5, 0x62, 0x01, 0x07, 0x5C, 0x00, 0x80, 0x1A, 0x06, 0x00, 0xEA, 0x07, 0x04, 0x04, 0x09, 0x1E,
	0x2A, 0x07, 0x32, 0x00, 0x00, 0x00, 0x40, 0xE2, 0x01, 0x00, 0x03, 0x01, 0x00, 0x0C, 0xCD, 0xB3,
	0x42, 0x53, 0x17, 0x86, 0x40, 0x15, 0x64, 0xAF, 0x00, 0x00, 0x08, 0x20, 0x00, 0x00, 0xDC, 0x05,
	0x00, 0x00, 0xF0, 0x0A, 0x00, 0x00, 0x32, 0x00, 0x00, 0x00, 0xE2, 0xFF, 0xFF, 0xFF, 0x0A, 0x00,
	0x00, 0x00, 0x3A, 0x00, 0x00, 0x00, 0x4E, 0x61, 0xBC, 0x00, 0xF4, 0x01, 0x00, 0x00, 0xE8, 0x03,
	0x00, 0x00, 0x79, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0xF0, 0xCE,
}

var monRFFrame = []byte{
	0xB5, 0x62, 0x0A, 0x38, 0x1C, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x01, 0x02, 0x01, 0x00, 0x00,
	0x00, 0x00, 0x64, 0x00, 0x00, 0x00, 0x88, 0x13, 0x00, 0x00, 0x19, 0x00, 0x80, 0x00, 0x82, 0x00,
	0x00, 0x00, 0x7D, 0x04,
}

func TestParseNavPVT(t *testing.T) {
	msg, frameLen, err := ParseFrame(navPVTFrame)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if frameLen != len(navPVTFrame) {
		t.Errorf("frameLen: got %d, want %d", frameLen, len(navPVTFrame))
	}

	pvt, ok := msg.(*NavPVT)
	if !ok {
		t.Fatalf("expected *NavPVT, got %T", msg)
	}

	if pvt.GetClassID() != NewClassID(ClassNAV, IDNavPVT) {
		t.Errorf("classID: got %v, want NAV-PVT", pvt.GetClassID())
	}
	if pvt.ITOW != 400000 {
		t.Errorf("iTOW: got %d, want 400000", pvt.ITOW)
	}
	if pvt.Year != 2026 {
		t.Errorf("year: got %d, want 2026", pvt.Year)
	}
	if pvt.Month != 4 || pvt.Day != 4 {
		t.Errorf("date: got %d/%d, want 4/4", pvt.Month, pvt.Day)
	}
	if pvt.Hour != 9 || pvt.Min != 30 || pvt.Sec != 42 {
		t.Errorf("time: got %d:%d:%d, want 9:30:42", pvt.Hour, pvt.Min, pvt.Sec)
	}
	if pvt.FixType != 3 {
		t.Errorf("fixType: got %d, want 3 (3D)", pvt.FixType)
	}
	if pvt.NumSV != 12 {
		t.Errorf("numSV: got %d, want 12", pvt.NumSV)
	}
	if !almostEqual(pvt.LatDeg(), 35.6550167) {
		t.Errorf("lat: got %f, want ~35.6550167", pvt.LatDeg())
	}
	if !almostEqual(pvt.LonDeg(), 139.6880333) {
		t.Errorf("lon: got %f, want ~139.6880333", pvt.LonDeg())
	}
	if !almostEqual(pvt.HMSLM(), 8.2) {
		t.Errorf("hMSL: got %f, want 8.2", pvt.HMSLM())
	}
	if !almostEqual(pvt.HAccM(), 1.5) {
		t.Errorf("hAcc: got %f, want 1.5", pvt.HAccM())
	}
	if !almostEqual(pvt.PDOPVal(), 1.21) {
		t.Errorf("pDOP: got %f, want 1.21", pvt.PDOPVal())
	}
}

func TestParseMonRF(t *testing.T) {
	msg, _, err := ParseFrame(monRFFrame)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}

	rf, ok := msg.(*MonRF)
	if !ok {
		t.Fatalf("expected *MonRF, got %T", msg)
	}

	if rf.GetClassID() != NewClassID(ClassMON, IDMonRF) {
		t.Errorf("classID: got %v, want MON-RF", rf.GetClassID())
	}
	if rf.NBlocks != 1 {
		t.Fatalf("nBlocks: got %d, want 1", rf.NBlocks)
	}

	b := rf.Blocks[0]
	if b.JammingState() != "OK" {
		t.Errorf("jamming state: got %q, want %q", b.JammingState(), "OK")
	}
	if b.JamInd != 25 {
		t.Errorf("jamInd: got %d, want 25", b.JamInd)
	}
	if b.AgcCnt != 5000 {
		t.Errorf("agcCnt: got %d, want 5000", b.AgcCnt)
	}
	if b.AntStatus != 2 {
		t.Errorf("antStatus: got %d, want 2", b.AntStatus)
	}
}

func TestChecksum(t *testing.T) {
	ckA, ckB := Checksum([]byte{0x01, 0x07, 0x00, 0x00})
	if ckA != 0x08 || ckB != 0x19 {
		t.Errorf("checksum: got 0x%02X 0x%02X, want 0x08 0x19", ckA, ckB)
	}
}

func TestChecksumMismatch(t *testing.T) {
	bad := make([]byte, len(navPVTFrame))
	copy(bad, navPVTFrame)
	bad[len(bad)-1] = 0xFF // corrupt checksum

	_, _, err := ParseFrame(bad)
	if err == nil {
		t.Fatal("expected checksum error, got nil")
	}
	if !errors.Is(err, ErrChecksumMismatch) {
		t.Errorf("expected ErrChecksumMismatch, got %v", err)
	}
}

func TestInvalidSyncBytes(t *testing.T) {
	data := []byte{0x00, 0x00, 0x01, 0x07, 0x00, 0x00, 0x08, 0x19}
	_, _, err := ParseFrame(data)
	if err == nil {
		t.Fatal("expected sync error, got nil")
	}
	if !errors.Is(err, ErrInvalidSyncBytes) {
		t.Errorf("expected ErrInvalidSyncBytes, got %v", err)
	}
}

func TestIncompleteFrame(t *testing.T) {
	// Only sync + class + id, no length or payload
	data := []byte{0xB5, 0x62, 0x01, 0x07}
	_, _, err := ParseFrame(data)
	if err == nil {
		t.Fatal("expected incomplete error, got nil")
	}
	if !errors.Is(err, ErrFrameIncomplete) {
		t.Errorf("expected ErrFrameIncomplete, got %v", err)
	}
}

func TestUnknownMessageReturnsRawMessage(t *testing.T) {
	// Build a frame with unknown class 0xFF, id 0xFF, empty payload
	frame := []byte{0xB5, 0x62, 0xFF, 0xFF, 0x00, 0x00}
	ckA, ckB := Checksum(frame[2:6])
	frame = append(frame, ckA, ckB)

	msg, _, err := ParseFrame(frame)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	raw, ok := msg.(*RawMessage)
	if !ok {
		t.Fatalf("expected *RawMessage, got %T", msg)
	}
	if raw.GetClassID() != NewClassID(0xFF, 0xFF) {
		t.Errorf("classID: got %v, want 0xFF-0xFF", raw.GetClassID())
	}
}

func TestClassIDString(t *testing.T) {
	tests := []struct {
		classID ClassID
		want    string
	}{
		{NewClassID(ClassNAV, IDNavPVT), "NAV-PVT"},
		{NewClassID(ClassRXM, IDRxmRAWX), "RXM-RAWX"},
		{NewClassID(ClassMON, IDMonRF), "MON-RF"},
		{NewClassID(ClassRXM, IDRxmSFRBX), "RXM-SFRBX"},
		{NewClassID(0xFF, 0xFF), "0xFF-0xFF"},
	}

	for _, tt := range tests {
		if got := tt.classID.String(); got != tt.want {
			t.Errorf("ClassID(%v).String() = %q, want %q", tt.classID, got, tt.want)
		}
	}
}

func TestDecoder(t *testing.T) {
	// Concatenate two frames with some garbage bytes in between
	var stream []byte
	stream = append(stream, 0x00, 0xFF, 0x24) // garbage (including NMEA '$')
	stream = append(stream, navPVTFrame...)
	stream = append(stream, 0x00, 0x00) // garbage
	stream = append(stream, monRFFrame...)

	dec := NewDecoder(bytes.NewReader(stream))

	// First message: NAV-PVT
	msg1, err := dec.Decode()
	if err != nil {
		t.Fatalf("decode 1: unexpected error: %v", err)
	}
	if _, ok := msg1.(*NavPVT); !ok {
		t.Fatalf("decode 1: expected *NavPVT, got %T", msg1)
	}

	// Second message: MON-RF
	msg2, err := dec.Decode()
	if err != nil {
		t.Fatalf("decode 2: unexpected error: %v", err)
	}
	if _, ok := msg2.(*MonRF); !ok {
		t.Fatalf("decode 2: expected *MonRF, got %T", msg2)
	}

	// EOF
	_, err = dec.Decode()
	if err != io.EOF {
		t.Errorf("decode 3: expected io.EOF, got %v", err)
	}
}

func TestCustomParser(t *testing.T) {
	called := false
	RegisterParser(ClassNAV, IDNavPVT, func(classID ClassID, payload []byte) (Message, error) {
		called = true
		return decodeNavPVT(classID, payload)
	})
	defer UnregisterParser(ClassNAV, IDNavPVT)

	_, _, err := ParseFrame(navPVTFrame)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if !called {
		t.Error("custom parser should have been called")
	}
}

func TestParseErrorStructured(t *testing.T) {
	bad := make([]byte, len(navPVTFrame))
	copy(bad, navPVTFrame)
	bad[len(bad)-1] = 0xFF

	_, _, err := ParseFrame(bad)
	if err == nil {
		t.Fatal("expected error")
	}

	var pe *ParseError
	if !errors.As(err, &pe) {
		t.Fatal("expected *ParseError")
	}
	if pe.Kind != ErrChecksum {
		t.Errorf("kind: got %d, want %d", pe.Kind, ErrChecksum)
	}
	if pe.Frame == nil {
		t.Error("frame should not be nil for checksum errors")
	}
}
