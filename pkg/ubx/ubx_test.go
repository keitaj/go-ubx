package ubx

import (
	"bytes"
	"encoding/binary"
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

// RXM-RAWX: 3 measurements (GPS SV01 L1, Galileo SV02 E1, QZSS SV193 L1)
var rxmRAWXFrame = []byte{
	0xB5, 0x62, 0x02, 0x15, 0x70, 0x00, 0x46, 0xB6, 0xF3, 0x7D, 0x00, 0x6A, 0x18, 0x41, 0x39, 0x09,
	0x12, 0x03, 0x01, 0x01, 0x00, 0x00, 0xA8, 0xC6, 0x4B, 0xB7, 0xAB, 0x43, 0x73, 0x41, 0x9E, 0xEF,
	0x27, 0x03, 0x43, 0x4D, 0x99, 0x41, 0x00, 0x50, 0x9A, 0xC4, 0x00, 0x01, 0x00, 0x00, 0x88, 0x13,
	0x2A, 0x02, 0x01, 0x03, 0x07, 0x00, 0xE9, 0x26, 0x31, 0x50, 0xC1, 0x5E, 0x76, 0x41, 0xBA, 0x49,
	0x0C, 0x54, 0x34, 0x6F, 0x9D, 0x41, 0x33, 0xF3, 0x0D, 0x44, 0x02, 0x02, 0x00, 0x00, 0xE0, 0x2E,
	0x26, 0x01, 0x01, 0x02, 0x03, 0x00, 0xCD, 0xCC, 0xCC, 0xC0, 0x1B, 0x7C, 0x82, 0x41, 0x00, 0x00,
	0x00, 0x62, 0x09, 0x47, 0xA8, 0x41, 0x66, 0x86, 0x5E, 0xC4, 0x05, 0xC1, 0x00, 0x00, 0x34, 0x21,
	0x2D, 0x01, 0x02, 0x02, 0x01, 0x00, 0x39, 0x16,
}

// RXM-SFRBX: GPS SV06, 10 navigation data words
var rxmSFRBXFrame = []byte{
	0xB5, 0x62, 0x02, 0x13, 0x30, 0x00, 0x00, 0x06, 0x00, 0x00, 0x0A, 0x03, 0x02, 0x00, 0x26, 0x4A,
	0xC3, 0x22, 0x4D, 0x3C, 0x2B, 0x1A, 0x1F, 0x00, 0xE8, 0x03, 0x34, 0x12, 0xAB, 0x00, 0x78, 0x56,
	0x34, 0x12, 0xF0, 0xDE, 0xBC, 0x9A, 0x44, 0x33, 0x22, 0x11, 0x88, 0x77, 0x66, 0x55, 0xDD, 0xCC,
	0xBB, 0xAA, 0x11, 0x00, 0xFF, 0xEE, 0x20, 0x21,
}

// NAV-SIG: 2 signals (GPS SV01 L1, Galileo SV05 E1)
var navSigFrame = []byte{
	0xB5, 0x62, 0x01, 0x43, 0x28, 0x00, 0x80, 0x1A, 0x06, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01,
	0x00, 0x00, 0x0C, 0x00, 0x2A, 0x07, 0x00, 0x01, 0x3D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x05,
	0x00, 0x00, 0xF8, 0xFF, 0x26, 0x04, 0x01, 0x03, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBF, 0xC7,
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

func TestParseNavSig(t *testing.T) {
	msg, frameLen, err := ParseFrame(navSigFrame)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}
	if frameLen != len(navSigFrame) {
		t.Errorf("frameLen: got %d, want %d", frameLen, len(navSigFrame))
	}

	sig, ok := msg.(*NavSig)
	if !ok {
		t.Fatalf("expected *NavSig, got %T", msg)
	}

	if sig.GetClassID() != NewClassID(ClassNAV, IDNavSig) {
		t.Errorf("classID: got %v, want NAV-SIG", sig.GetClassID())
	}
	if sig.ITOW != 400000 {
		t.Errorf("iTOW: got %d, want 400000", sig.ITOW)
	}
	if sig.NumSigs != 2 {
		t.Fatalf("numSigs: got %d, want 2", sig.NumSigs)
	}
	if len(sig.Signals) != 2 {
		t.Fatalf("signals len: got %d, want 2", len(sig.Signals))
	}

	// Signal 0: GPS SV01 L1C/A
	s0 := sig.Signals[0]
	if s0.GnssID != GnssIDGPS {
		t.Errorf("sig[0] gnssId: got %d, want %d (GPS)", s0.GnssID, GnssIDGPS)
	}
	if s0.SvID != 1 {
		t.Errorf("sig[0] svId: got %d, want 1", s0.SvID)
	}
	if s0.SigID != 0 {
		t.Errorf("sig[0] sigId: got %d, want 0", s0.SigID)
	}
	if s0.CNO != 42 {
		t.Errorf("sig[0] cno: got %d, want 42", s0.CNO)
	}
	if s0.QualityInd != 7 {
		t.Errorf("sig[0] qualityInd: got %d, want 7", s0.QualityInd)
	}
	if s0.QualityStr() != "code+carrier locked (3)" {
		t.Errorf("sig[0] qualityStr: got %q", s0.QualityStr())
	}
	if s0.IonoModel != 1 {
		t.Errorf("sig[0] ionoModel: got %d, want 1 (Klobuchar)", s0.IonoModel)
	}
	if !almostEqual(s0.PrResM(), 1.2) {
		t.Errorf("sig[0] prRes: got %f, want 1.2", s0.PrResM())
	}
	if s0.Health() != 1 {
		t.Errorf("sig[0] health: got %d, want 1 (healthy)", s0.Health())
	}
	if !s0.PrSmoothed() {
		t.Error("sig[0] should have prSmoothed")
	}
	if !s0.PrUsed() {
		t.Error("sig[0] should have prUsed")
	}
	if !s0.CrUsed() {
		t.Error("sig[0] should have crUsed")
	}
	if !s0.DoUsed() {
		t.Error("sig[0] should have doUsed")
	}

	// Signal 1: Galileo SV05 E1
	s1 := sig.Signals[1]
	if s1.GnssID != GnssIDGalileo {
		t.Errorf("sig[1] gnssId: got %d, want %d (Galileo)", s1.GnssID, GnssIDGalileo)
	}
	if s1.SvID != 5 {
		t.Errorf("sig[1] svId: got %d, want 5", s1.SvID)
	}
	if s1.CNO != 38 {
		t.Errorf("sig[1] cno: got %d, want 38", s1.CNO)
	}
	if s1.QualityInd != 4 {
		t.Errorf("sig[1] qualityInd: got %d, want 4", s1.QualityInd)
	}
	if s1.CorrSource != 1 {
		t.Errorf("sig[1] corrSource: got %d, want 1 (SBAS)", s1.CorrSource)
	}
	if s1.IonoModel != 3 {
		t.Errorf("sig[1] ionoModel: got %d, want 3 (dual-freq)", s1.IonoModel)
	}
	if !almostEqual(s1.PrResM(), -0.8) {
		t.Errorf("sig[1] prRes: got %f, want -0.8", s1.PrResM())
	}
	if s1.Health() != 1 {
		t.Errorf("sig[1] health: got %d, want 1", s1.Health())
	}
	if !s1.PrUsed() {
		t.Error("sig[1] should have prUsed")
	}
	if s1.CrUsed() {
		t.Error("sig[1] should not have crUsed")
	}
	if s1.DoUsed() {
		t.Error("sig[1] should not have doUsed")
	}
}

func TestNavSigPayloadTooShort(t *testing.T) {
	// Build a frame claiming 2 signals but only 1 signal worth of data
	payload := make([]byte, 24) // header(8) + 1 signal(16), but numSigs=2
	binary.LittleEndian.PutUint32(payload[0:4], 400000)
	payload[5] = 2 // numSigs = 2, but only space for 1

	frame := make([]byte, 0, 32)
	frame = append(frame, 0xB5, 0x62, 0x01, 0x43)
	lenBytes := make([]byte, 2)
	binary.LittleEndian.PutUint16(lenBytes, uint16(len(payload)))
	frame = append(frame, lenBytes...)
	frame = append(frame, payload...)
	var ckA, ckB byte
	for _, b := range frame[2:] {
		ckA += b
		ckB += ckA
	}
	frame = append(frame, ckA, ckB)

	_, _, err := ParseFrame(frame)
	if err == nil {
		t.Fatal("expected payload length error")
	}
	if !errors.Is(err, ErrInvalidPayload) {
		t.Errorf("expected ErrInvalidPayload, got %v", err)
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

func TestParseRxmRAWX(t *testing.T) {
	msg, _, err := ParseFrame(rxmRAWXFrame)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}

	rawx, ok := msg.(*RxmRAWX)
	if !ok {
		t.Fatalf("expected *RxmRAWX, got %T", msg)
	}

	if rawx.GetClassID() != NewClassID(ClassRXM, IDRxmRAWX) {
		t.Errorf("classID: got %v, want RXM-RAWX", rawx.GetClassID())
	}
	if rawx.Week != 2361 {
		t.Errorf("week: got %d, want 2361", rawx.Week)
	}
	if rawx.LeapS != 18 {
		t.Errorf("leapS: got %d, want 18", rawx.LeapS)
	}
	if rawx.NumMeas != 3 {
		t.Fatalf("numMeas: got %d, want 3", rawx.NumMeas)
	}
	if len(rawx.Meas) != 3 {
		t.Fatalf("meas len: got %d, want 3", len(rawx.Meas))
	}

	// Measurement 0: GPS SV01 L1
	m0 := rawx.Meas[0]
	if m0.GnssID != GnssIDGPS {
		t.Errorf("meas[0] gnssId: got %d, want %d (GPS)", m0.GnssID, GnssIDGPS)
	}
	if m0.SvID != 1 {
		t.Errorf("meas[0] svId: got %d, want 1", m0.SvID)
	}
	if !almostEqual(m0.PrMes, 20200123.456) {
		t.Errorf("meas[0] prMes: got %f, want 20200123.456", m0.PrMes)
	}
	if !almostEqual(m0.CpMes, 106123456.789) {
		t.Errorf("meas[0] cpMes: got %f, want 106123456.789", m0.CpMes)
	}
	if m0.CNO != 42 {
		t.Errorf("meas[0] cno: got %d, want 42", m0.CNO)
	}
	if !m0.PrValid() {
		t.Error("meas[0] should have valid pseudorange")
	}
	if !m0.CpValid() {
		t.Error("meas[0] should have valid carrier phase")
	}
	if !m0.HalfCyc() {
		t.Error("meas[0] should have half cycle flag")
	}

	// Measurement 1: Galileo SV02 E1
	m1 := rawx.Meas[1]
	if m1.GnssID != GnssIDGalileo {
		t.Errorf("meas[1] gnssId: got %d, want %d (Galileo)", m1.GnssID, GnssIDGalileo)
	}
	if m1.SvID != 2 {
		t.Errorf("meas[1] svId: got %d, want 2", m1.SvID)
	}
	if m1.CNO != 38 {
		t.Errorf("meas[1] cno: got %d, want 38", m1.CNO)
	}
	if m1.HalfCyc() {
		t.Error("meas[1] should not have half cycle flag")
	}

	// Measurement 2: QZSS SV193
	m2 := rawx.Meas[2]
	if m2.GnssID != GnssIDQZSS {
		t.Errorf("meas[2] gnssId: got %d, want %d (QZSS)", m2.GnssID, GnssIDQZSS)
	}
	if m2.SvID != 193 {
		t.Errorf("meas[2] svId: got %d, want 193", m2.SvID)
	}
	if m2.CNO != 45 {
		t.Errorf("meas[2] cno: got %d, want 45", m2.CNO)
	}
	if !m2.PrValid() {
		t.Error("meas[2] should have valid pseudorange")
	}
	if m2.CpValid() {
		t.Error("meas[2] should not have valid carrier phase")
	}
}

func TestParseRxmSFRBX(t *testing.T) {
	msg, _, err := ParseFrame(rxmSFRBXFrame)
	if err != nil {
		t.Fatalf("unexpected error: %v", err)
	}

	sfrbx, ok := msg.(*RxmSFRBX)
	if !ok {
		t.Fatalf("expected *RxmSFRBX, got %T", msg)
	}

	if sfrbx.GetClassID() != NewClassID(ClassRXM, IDRxmSFRBX) {
		t.Errorf("classID: got %v, want RXM-SFRBX", sfrbx.GetClassID())
	}
	if sfrbx.GnssID != GnssIDGPS {
		t.Errorf("gnssId: got %d, want %d (GPS)", sfrbx.GnssID, GnssIDGPS)
	}
	if sfrbx.SvID != 6 {
		t.Errorf("svId: got %d, want 6", sfrbx.SvID)
	}
	if sfrbx.NumWords != 10 {
		t.Fatalf("numWords: got %d, want 10", sfrbx.NumWords)
	}
	if len(sfrbx.Dwrd) != 10 {
		t.Fatalf("dwrd len: got %d, want 10", len(sfrbx.Dwrd))
	}
	if sfrbx.Channel != 3 {
		t.Errorf("channel: got %d, want 3", sfrbx.Channel)
	}
	if sfrbx.Version != 2 {
		t.Errorf("version: got %d, want 2", sfrbx.Version)
	}
	if sfrbx.Dwrd[0] != 0x22C34A26 {
		t.Errorf("dwrd[0]: got 0x%08X, want 0x22C34A26", sfrbx.Dwrd[0])
	}
	if sfrbx.Dwrd[9] != 0xEEFF0011 {
		t.Errorf("dwrd[9]: got 0x%08X, want 0xEEFF0011", sfrbx.Dwrd[9])
	}
}

func TestDecoderAllMessageTypes(t *testing.T) {
	var stream []byte
	stream = append(stream, navPVTFrame...)
	stream = append(stream, navSigFrame...)
	stream = append(stream, rxmRAWXFrame...)
	stream = append(stream, monRFFrame...)
	stream = append(stream, rxmSFRBXFrame...)

	dec := NewDecoder(bytes.NewReader(stream))

	types := []string{"NAV-PVT", "NAV-SIG", "RXM-RAWX", "MON-RF", "RXM-SFRBX"}
	for i, want := range types {
		msg, err := dec.Decode()
		if err != nil {
			t.Fatalf("decode %d: unexpected error: %v", i, err)
		}
		got := msg.GetClassID().String()
		if got != want {
			t.Errorf("decode %d: got %s, want %s", i, got, want)
		}
	}

	_, err := dec.Decode()
	if err != io.EOF {
		t.Errorf("expected io.EOF, got %v", err)
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
		{NewClassID(ClassNAV, IDNavSig), "NAV-SIG"},
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
