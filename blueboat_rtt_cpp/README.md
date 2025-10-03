# BlueBoat Micron Data Modem — RTT (addressed) demo in C++

> **Heads-up:** Application-level RTT is **coarse** (expect multi-meter errors). For sub-meter ranging, use the modem's `rng` or a USBL (MicronNav).

## Build
```bash
make
# or: g++ -std=c++17 -O2 -pthread blueboat_rtt.cpp -o blueboat_rtt
```

## Run (one per boat)
```bash
# Boat 1 (master)
./blueboat_rtt --id 1 --port /dev/ttyUSB0 --c 1500 --offset-ms 0.0

# Boat 2
./blueboat_rtt --id 2 --port /dev/ttyUSB0 --c 1500 --offset-ms 0.0

# Boat 3
./blueboat_rtt --id 3 --port /dev/ttyUSB0 --c 1500 --offset-ms 0.0
```

**Args**
- `--id`        : 1, 2, or 3
- `--port`      : serial device path (e.g., `/dev/ttyUSB0`)
- `--baud`      : default 9600
- `--c`         : speed of sound m/s (e.g., 1482 freshwater @20°C; 1500–1530 sea)
- `--offset-ms` : calibration offset (ms) subtracted from RTT before halving
- `--repeats`   : min-of-N repeats per measurement (default 5)
- `--cycle`     : TDMA cycle length (s, default 12)
- `--timeout`   : per-PING timeout waiting for PONG (s, default 3)

## What it does
- ID=1 measures and broadcasts `d12` and `d13`.
- ID=1 asks ID=2 via `CMD` (0x31) to measure `d23` and broadcast it.
- All messages are compactly framed: `MAGIC 0xA5 | LEN | TYPE | PAYLOAD | CRC8`.
- A background listener thread:
  - replies immediately to addressed `PING` (PONG),
  - updates local distances on `DIST` broadcast,
  - executes command `0x31` on ID=2 (measure d23).

## Calibration (optional)
1. Place two modems at a **known short** distance (≈1 m) in **water**.
2. Run 20–50 measures; take **min-of-N** RTT.
3. Compute `offset_ms = (RTT_min - 2*d_ref/c)*1000`.
4. Pass it as `--offset-ms` to reduce bias.

## Notes
- Only **one** acoustic exchange at a time (TDMA) to avoid collisions.
- This demo is **addressed** at application layer (PING includes SRC/DST IDs).
- The modem layer has **no addressing, no ARQ**; we add CRC8 and matching by SEQ.

**License:** MIT
