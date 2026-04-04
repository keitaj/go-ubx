# go-ubx

UBX binary protocol parser for u-blox GNSS receivers in Go.

## Supported Messages

| Message | Class | ID | Description |
|---------|-------|----|-------------|
| NAV-PVT | 0x01 | 0x07 | Position, Velocity, Time |
| RXM-RAWX | 0x02 | 0x15 | Multi-GNSS Raw Measurements |
| MON-RF | 0x0A | 0x38 | RF Information (jamming/spoofing) |
| RXM-SFRBX | 0x02 | 0x13 | Raw Subframe Data |

Unknown messages are returned as `*RawMessage` without error.

## Usage

```go
package main

import (
	"fmt"
	"io"
	"os"

	"github.com/keitaj/go-ubx/pkg/ubx"
)

func main() {
	f, _ := os.Open("data.ubx")
	defer f.Close()

	dec := ubx.NewDecoder(f)
	for {
		msg, err := dec.Decode()
		if err == io.EOF {
			break
		}
		if err != nil {
			fmt.Println("error:", err)
			continue
		}

		switch m := msg.(type) {
		case *ubx.NavPVT:
			fmt.Printf("PVT: %.7f, %.7f fix=%d sats=%d\n",
				m.LatDeg(), m.LonDeg(), m.FixType, m.NumSV)
		case *ubx.MonRF:
			for _, b := range m.Blocks {
				fmt.Printf("RF: jamming=%s jamInd=%d\n",
					b.JammingState(), b.JamInd)
			}
		case *ubx.RxmRAWX:
			fmt.Printf("RAWX: %d measurements, week=%d\n",
				m.NumMeas, m.Week)
		}
	}
}
```

## Custom Parsers

```go
ubx.RegisterParser(0x01, 0x03, func(classID ubx.ClassID, payload []byte) (ubx.Message, error) {
	// decode your custom message
})
defer ubx.UnregisterParser(0x01, 0x03)
```

## Install

```
go get github.com/keitaj/go-ubx
```

## License

MIT
