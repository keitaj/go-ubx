<!-- version: 1.0.0 -->

This file provides guidance to AI agents when working with code in the go-ubx repository.

## Project Overview

go-ubx is a Go library for encoding and decoding the u-blox UBX binary protocol used by GNSS receivers (e.g., ZED-F9P). It provides a streaming decoder, typed message structs with helper methods, and a CLI capture tool (`ubxcap`). Pure Go, no external dependencies.

## Principles

- Follow existing patterns in the surrounding code
- Write tests for new functionality (embed test frames as byte arrays)
- Keep changes focused — one message type or feature per PR
- Verify constants (class/ID, key IDs) against the u-blox F9 interface description
- Do not introduce external dependencies without strong justification

## Commands

### Build

```bash
go build ./...                    # Build all packages
go build ./cmd/ubxcap             # Build the capture tool
```

### Test

```bash
go test ./...                     # All tests
go test -v -race ./...            # Verbose with race detection (CI default)
go test -run TestName ./pkg/ubx/  # Specific test
```

### Lint & Format

```bash
go vet ./...                      # Static analysis
gofmt -l .                        # List files needing formatting (CI rejects any)
gofmt -w .                        # Auto-format all files
```

## Architecture

### Core Library (`pkg/ubx/`)

| File          | Purpose                                                     |
| ------------- | ----------------------------------------------------------- |
| `ubx.go`      | Message types, constants, class/ID definitions, decoders    |
| `decoder.go`  | Streaming frame decoder (sync, buffer, checksum validation) |
| `encode.go`   | Frame encoder, CFG-VALSET/VALGET builders, config key constants |
| `errors.go`   | Structured `ParseError` with kinds (`ErrChecksum`, `ErrPayloadLen`, etc.) |

**Supported messages:** NAV-PVT, NAV-SAT, NAV-SIG, RXM-RAWX, RXM-SFRBX, MON-RF, ACK-ACK, ACK-NAK, CFG-VALGET, CFG-VALSET. Unknown messages decode as `*RawMessage`.

### Capture Tool (`cmd/ubxcap/`)

| File                       | Purpose                              |
| -------------------------- | ------------------------------------ |
| `main.go`                  | CLI entry point, receiver config, logging |
| `serialport/serial_*.go`   | Platform-specific serial port (darwin, linux) |

### Key Patterns

- **Message interface**: All message types implement `Message` with `GetClassID() ClassID`.
- **Bitfield helpers**: Methods like `SvUsed()`, `Health()`, `QualityInd()` extract bits from flags fields. Name clearly indicates the return type (bool for single-bit, uint8 for multi-bit).
- **Unit conversion methods**: `LatDeg()`, `HAccM()`, `PrResM()` — always name with the target unit suffix.
- **Decoder registration**: `RegisterParser()` / `UnregisterParser()` for custom message types, protected by `sync.RWMutex`.
- **Builder pattern**: `CfgValsetBuilder` for constructing multi-key configuration frames.
- **Fletcher-8 checksum**: Computed over class + ID + length + payload bytes.
- **Pre-allocated errors**: `errIncomplete` avoids heap allocations on the hot path.

## Adding a New Message Type

1. Add `ID<MsgName>` constant and `classIDNames` entry in `ubx.go`
2. Define the message struct implementing `Message` interface
3. Add helper methods for bitfield extraction and unit conversion
4. Add `decode<MsgName>()` function and wire it into `decodeMessage()` switch
5. Add config key constant in `encode.go` if applicable (e.g., `KeyMsgout<MsgName>USB`)
6. Add tests: round-trip decode with known-good frame bytes, short-payload error cases, helper method verification
7. Update `README.md`: add the new message to the supported messages table and add usage examples if the message introduces new patterns
8. Verify all constants against the u-blox interface description document

## Testing Conventions

- Test frames are embedded as `var <name>Frame = []byte{...}` hex arrays (not external files)
- Float comparisons use `almostEqual()` helper with `0.0001` tolerance
- Type assertions use `msg.(*ExpectedType)` with `t.Fatalf()` on failure
- Error tests verify `errors.Is(err, ErrInvalidPayload)` etc.
- Test naming: `TestParse<Msg>` for decode, `TestEncode<Msg>` for encode, `Test<Msg>PayloadTooShort` for error paths

## CI

GitHub Actions on push/PR to `main`:
- **test**: `go vet` + `go test -v -race -coverprofile` + `go build ./cmd/...`
- **lint**: `go vet` + `gofmt -l .` (zero tolerance for formatting)

## Key Notes

- **UBX frame format**: `[0xB5 0x62] [class] [id] [len LE16] [payload...] [ck_a ck_b]`
- **Max payload**: 65,535 bytes. Header overhead: 8 bytes.
- **Config key format**: `0xSSKKKKKK` where SS = size/type, KKKKKK = key ID.
- **GNSS IDs**: GPS=0, SBAS=1, Galileo=2, BeiDou=3, IMES=4, QZSS=5, GLONASS=6.
- **Serial port**: Platform-specific implementations (darwin/linux only, no Windows).
- **Thread safety**: Decoder instances are NOT thread-safe (one per reader). Custom parser registry IS thread-safe.
