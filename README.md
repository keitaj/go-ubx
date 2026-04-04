# go-ubx

UBX binary protocol library for u-blox GNSS receivers in Go.

## Supported Messages

| Message | Class | ID | Direction | Description |
|---------|-------|----|-----------|-------------|
| NAV-PVT | 0x01 | 0x07 | Rx | Position, Velocity, Time |
| RXM-RAWX | 0x02 | 0x15 | Rx | Multi-GNSS Raw Measurements |
| MON-RF | 0x0A | 0x38 | Rx | RF Information (jamming/spoofing) |
| RXM-SFRBX | 0x02 | 0x13 | Rx | Raw Subframe Data |
| ACK-ACK | 0x05 | 0x01 | Rx | Acknowledgment |
| ACK-NAK | 0x05 | 0x00 | Rx | Negative Acknowledgment |
| CFG-VALGET | 0x06 | 0x8B | Tx/Rx | Configuration Get (poll + response) |
| CFG-VALSET | 0x06 | 0x8A | Tx | Configuration Set |

Unknown messages are returned as `*RawMessage` without error.

## Decoding

```go
dec := ubx.NewDecoder(serialPort)
for {
    msg, err := dec.Decode()
    if err == io.EOF {
        break
    }
    if err != nil {
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
    case *ubx.AckAck:
        fmt.Printf("ACK: %s\n", m.AckedClassID())
    case *ubx.AckNak:
        fmt.Printf("NAK: %s\n", m.NakedClassID())
    case *ubx.CfgValget:
        rate, _ := m.GetU1(ubx.KeyMsgoutNavPvtUSB)
        fmt.Printf("NAV-PVT rate: %d\n", rate)
    }
}
```

## Configuring the Receiver

```go
// Enable messages on USB port
frame := ubx.NewCfgValset(ubx.LayerRAM).
    AddU1(ubx.KeyMsgoutNavPvtUSB, 1).
    AddU1(ubx.KeyMsgoutRxmRawxUSB, 1).
    AddU1(ubx.KeyMsgoutMonRfUSB, 1).
    Build()
serialPort.Write(frame)

// Set measurement rate to 10Hz
serialPort.Write(ubx.SetMeasurementRate(100))

// Read current configuration
serialPort.Write(ubx.PollCfgValget(ubx.PollLayerRAM,
    ubx.KeyMsgoutNavPvtUSB,
    ubx.KeyRateMeas,
))
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
