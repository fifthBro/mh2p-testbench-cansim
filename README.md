# mh2p-testbench-cansim

Arduino-based bus simulator that reproduces the CAN behaviour on VAG PCM (MIB / MH2p / PCM5) systems.

Its purpose is **not** to simulate a whole vehicle, but to  reproduce the ** CAN messages** so that a real PCM and Gateway can boot and operate correctly on a test bench.


## 🎯 What mh2p-testbench-cansim does

- Replays **bit-for-bit equivalent CAN frames** observed from real system
- Supports:
  - Standard (11-bit) CAN frames
  - Extended (29-bit) CAN frames
- Matches:
  - CAN IDs
  - Payloads
  - Timing / periodicity
- Works with:
  - Real PCM
  - Real Gateway
  - Bench or vehicle harnesses
- Designed to survive **strict integrity / timing checks**

Based on **real CAN logs captured from an original car**, but can work both:
- standalone
- and connected to PCM + Gateway

---

## ❌ What it does *not* do

- It does **not** simulate the entire vehicle
- It does **not** generate BAP 
- It does **not** reverse or dump OEM firmware
- It does **not** emulate UDS, diagnostics, or flashing

It only emulates **what basic startup that real system puts on the bus**.

---

## 🧠 Why this exists

The real system uses complex network of various Can nodes and is inconvenient for experimentation

However, some of the CAN behaviour is **deterministic** so this sim reproduces it using inexpensive, open hardware.


## 🧩 Supported systems (observed)

- Porsche PCM5 / MH2P
- Gateway + PCM bench setups
- Infotainment CAN (CAN-MIB)

> ⚠️ Compatibility depends on firmware and vehicle generation.  
> This project reflects **observed behaviour**, not official specs.


## 🛠 Hardware requirements

- Arduino Uno / Nano / compatible
- MCP2515-based CAN shield (e.g. Seeed CAN Shield v2.0)
- 120 Ω CAN termination (as required by your bench)
- Bench power supply (12 V)


## ⚙️ CAN configuration

- CAN speed: **500 kbps**
- Uses:
  - `mcp2515_can` library (Seeed / Cory Fowler)
- Extended frames are explicitly enabled


## 🚀 Usage

1. Flash the sketch to your Arduino
2. Connect CAN-H / CAN-L to the Infotainment CAN bus
3. Power PCM + Gateway
4. it will automatically begin transmitting  traffic
5. PCM should boot and behave as if the OEM nodes were present

No runtime interaction is required.

## 📎 Disclaimer

This project is **not affiliated with any car manufacturer**.

All trademarks belong to their respective owners.
