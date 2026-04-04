// ubxcap connects to a u-blox GNSS receiver, enables UBX messages
// (NAV-PVT, NAV-SIG, RXM-RAWX, MON-RF, RXM-SFRBX), and logs
// received data to stdout and optionally to a file.
//
// Usage:
//
//	ubxcap -port /dev/ttyACM0
//	ubxcap -port /dev/ttyACM0 -baud 460800 -out capture.ubx
//	ubxcap -port /dev/ttyACM0 -rate 100  # 10Hz
package main

import (
	"encoding/hex"
	"flag"
	"fmt"
	"io"
	"log"
	"os"
	"os/signal"
	"path/filepath"
	"strings"
	"sync/atomic"
	"syscall"

	"github.com/keitaj/go-ubx/cmd/ubxcap/serialport"
	"github.com/keitaj/go-ubx/pkg/ubx"
)

func main() {
	portName := flag.String("port", "", "Serial port (e.g., /dev/ttyACM0, /dev/cu.usbmodem*)")
	baudRate := flag.Int("baud", 115200, "Baud rate")
	outFile := flag.String("out", "", "Output file for raw UBX binary (optional)")
	measRate := flag.Int("rate", 1000, "Measurement interval in ms (1000=1Hz, 100=10Hz)")
	rawOnly := flag.Bool("raw", false, "Save raw bytes only, no console output")
	flag.Parse()

	if *measRate <= 0 {
		log.Fatalf("Invalid measurement rate: %d ms (must be > 0)", *measRate)
	}

	if *portName == "" {
		*portName = detectPort()
		if *portName == "" {
			fmt.Fprintln(os.Stderr, "Usage: ubxcap -port /dev/ttyACM0")
			fmt.Fprintln(os.Stderr, "\nTip: look for /dev/ttyACM* (Linux) or /dev/cu.usbmodem* (macOS)")
			os.Exit(1)
		}
		fmt.Fprintf(os.Stderr, "Auto-detected port: %s\n", *portName)
	}

	port, err := serialport.Open(*portName, *baudRate)
	if err != nil {
		log.Fatalf("Failed to open %s: %v", *portName, err)
	}
	defer port.Close()

	fmt.Fprintf(os.Stderr, "Connected to %s @ %d baud\n", *portName, *baudRate)

	// Configure receiver
	configure(port, *measRate)

	// Open output file if specified
	var outFile_ *os.File
	var reader io.Reader = port
	if *outFile != "" {
		f, err := os.Create(*outFile)
		if err != nil {
			log.Fatalf("Failed to create %s: %v", *outFile, err)
		}
		defer f.Close()
		outFile_ = f
		reader = io.TeeReader(port, f)
		fmt.Fprintf(os.Stderr, "Saving raw data to %s\n", *outFile)
	}

	// Handle Ctrl+C: close the port so Decode() returns io.EOF,
	// then main exits normally and deferred cleanup (including file
	// close/flush) runs.
	sig := make(chan os.Signal, 1)
	signal.Notify(sig, syscall.SIGINT, syscall.SIGTERM)

	fmt.Fprintln(os.Stderr, "Receiving... (Ctrl+C to stop)")
	fmt.Fprintln(os.Stderr)

	dec := ubx.NewDecoder(reader)
	var msgCount atomic.Int64

	go func() {
		<-sig
		port.Close()
	}()

	for {
		msg, err := dec.Decode()
		if err != nil {
			break
		}
		msgCount.Add(1)

		if *rawOnly {
			continue
		}

		printMessage(msg)
	}

	// Sync output file before deferred Close to ensure all data is flushed.
	if outFile_ != nil {
		outFile_.Sync()
	}
	fmt.Fprintf(os.Stderr, "\n\nReceived %d messages total\n", msgCount.Load())
}

func configure(port io.Writer, measRateMs int) {
	fmt.Fprintln(os.Stderr, "Configuring receiver...")

	frame := ubx.NewCfgValset(ubx.LayerRAM).
		AddU1(ubx.KeyMsgoutNavPvtUSB, 1).
		AddU1(ubx.KeyMsgoutNavSigUSB, 1).
		AddU1(ubx.KeyMsgoutRxmRawxUSB, 1).
		AddU1(ubx.KeyMsgoutMonRfUSB, 1).
		AddU1(ubx.KeyMsgoutRxmSfrbxUSB, 1).
		AddU2(ubx.KeyRateMeas, uint16(measRateMs)).
		Build()

	if _, err := port.Write(frame); err != nil {
		log.Fatalf("Failed to send configuration: %v", err)
	}

	fmt.Fprintf(os.Stderr, "  NAV-PVT, NAV-SIG, RXM-RAWX, MON-RF, RXM-SFRBX enabled\n")
	fmt.Fprintf(os.Stderr, "  Measurement rate: %dms (%dHz)\n", measRateMs, 1000/measRateMs)
}

func printMessage(msg ubx.Message) {
	switch m := msg.(type) {
	case *ubx.NavPVT:
		fixStr := fixString(m)
		fmt.Printf("[NAV-PVT] %04d-%02d-%02d %02d:%02d:%02d | fix=%-14s | sats=%2d | %.7f, %.7f | alt=%.1fm | hAcc=%.2fm | pDOP=%.2f\n",
			m.Year, m.Month, m.Day, m.Hour, m.Min, m.Sec,
			fixStr, m.NumSV,
			m.LatDeg(), m.LonDeg(), m.HMSLM(),
			m.HAccM(), m.PDOPVal())

	case *ubx.NavSig:
		fmt.Printf("[NAV-SIG] %d signals\n", m.NumSigs)
		for i, s := range m.Signals {
			gnss := gnssName(s.GnssID)
			health := "?"
			switch s.Health() {
			case 1:
				health = "OK"
			case 2:
				health = "NG"
			}
			fmt.Printf("  [%2d] %s SV%3d sig=%d | CN0=%2d dB-Hz | quality=%d (%s) | health=%s | prUsed=%v crUsed=%v\n",
				i, gnss, s.SvID, s.SigID, s.CNO, s.QualityInd, s.QualityStr(), health, s.PrUsed(), s.CrUsed())
		}

	case *ubx.RxmRAWX:
		fmt.Printf("[RXM-RAWX] week=%d tow=%.3fs | %d measurements\n",
			m.Week, m.RcvTow, m.NumMeas)
		for i, meas := range m.Meas {
			gnss := gnssName(meas.GnssID)
			pr := "-"
			if meas.PrValid() {
				pr = fmt.Sprintf("%.3f", meas.PrMes)
			}
			fmt.Printf("  [%2d] %s SV%3d sig=%d | CN0=%2d dB-Hz | PR=%s m\n",
				i, gnss, meas.SvID, meas.SigID, meas.CNO, pr)
		}

	case *ubx.MonRF:
		for _, b := range m.Blocks {
			fmt.Printf("[MON-RF] block=%d | jamming=%-8s | jamInd=%3d | agc=%5d | ant=%d/%d\n",
				b.BlockID, b.JammingState(), b.JamInd, b.AgcCnt, b.AntStatus, b.AntPower)
		}

	case *ubx.RxmSFRBX:
		gnss := gnssName(m.GnssID)
		n := len(m.Dwrd)
		if n > 3 {
			n = 3
		}
		words := make([]string, n)
		for i := 0; i < n; i++ {
			words[i] = fmt.Sprintf("%08X", m.Dwrd[i])
		}
		fmt.Printf("[RXM-SFRBX] %s SV%d | %d words | %s...\n",
			gnss, m.SvID, m.NumWords, strings.Join(words, " "))

	case *ubx.AckAck:
		fmt.Printf("[ACK-ACK] %s accepted\n", m.AckedClassID())

	case *ubx.AckNak:
		fmt.Printf("[ACK-NAK] %s rejected\n", m.NakedClassID())

	case *ubx.CfgValget:
		fmt.Printf("[CFG-VALGET] layer=%d | %d key-value pairs\n", m.Layer, len(m.KeyVals))
		for _, kv := range m.KeyVals {
			fmt.Printf("  key=0x%08X val=%s\n", kv.Key, hex.EncodeToString(kv.Val))
		}

	case *ubx.RawMessage:
		fmt.Printf("[%s] %d bytes payload\n", m.GetClassID(), len(m.Payload))
	}
}

func fixString(m *ubx.NavPVT) string {
	s := "No fix"
	switch m.FixType {
	case 2:
		s = "2D"
	case 3:
		s = "3D"
	case 4:
		s = "GNSS+DR"
	case 5:
		s = "Time only"
	}
	if m.Flags&0x01 != 0 {
		switch (m.Flags >> 6) & 0x03 {
		case 1:
			s += "/Float"
		case 2:
			s += "/Fixed"
		}
	}
	return s
}

func gnssName(id uint8) string {
	switch id {
	case ubx.GnssIDGPS:
		return "GPS    "
	case ubx.GnssIDSBAS:
		return "SBAS   "
	case ubx.GnssIDGalileo:
		return "Galileo"
	case ubx.GnssIDBeiDou:
		return "BeiDou "
	case ubx.GnssIDQZSS:
		return "QZSS   "
	case ubx.GnssIDGLONASS:
		return "GLONASS"
	default:
		return fmt.Sprintf("GNSS(%d)", id)
	}
}

func detectPort() string {
	// Linux: /dev/ttyACM*
	matches, _ := filepath.Glob("/dev/ttyACM*")
	if len(matches) > 0 {
		return matches[0]
	}
	// macOS: /dev/cu.usbmodem*
	matches, _ = filepath.Glob("/dev/cu.usbmodem*")
	if len(matches) > 0 {
		return matches[0]
	}
	return ""
}
