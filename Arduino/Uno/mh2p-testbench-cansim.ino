/*
 * VAG PCM (MIB / MH2p / PCM5) canbus simulator ( High Bursts) for Arduino UNO
 * -----------------------------------------------------------
 * - Hardware: Arduino UNO + Seeed CAN-BUS Shield v2.0 (MCP2515)
 * - Bitrate: 500 kbps
 *
 * Behaviour:
 *   • PHASE_INIT (0–3 s):
 *       - Light standard traffic
 *       - 3C0 pattern active
 *   • PHASE_BOOTLOADER (3–9 s):
 *       - High-frequency extended bootloader bursts (every 200 ms)
 *       - Standard traffic continues
 *       - 3C0 active
 *   • PHASE_RUNTIME (9–20 s):
 *       - Only standard frames & 3C0
 *       - No extended bursts
 *   • After 20 s: cycle restarts
 *
 * Notes:
 *   - Standard payloads are taken from real system.
 *   - 3C0 cycles through C0/DE/2E patterns with a semi-realistic timing sequence.
 */

#include <SPI.h>
#include "mcp2515_can.h"

// ============================
// CAN SHIELD SETUP
// ============================

const int SPI_CS_PIN = 9;          // Seeed CAN-BUS Shield v2.0 default CS
mcp2515_can CAN(SPI_CS_PIN);       // MCP2515 driver instance

// ============================
// PHASE STATE MACHINE
// ============================

enum BootPhase {
  PHASE_INIT = 0,
  PHASE_BOOTLOADER,
  PHASE_RUNTIME
};

BootPhase currentPhase = PHASE_INIT;
unsigned long phaseStartMs = 0;

const unsigned long INIT_DURATION_MS       = 3000;   // 0–3 s
const unsigned long BOOTLOADER_DURATION_MS = 6000;   // 3–9 s
const unsigned long CYCLE_DURATION_MS      = 20000;  // total cycle 0–20 s

// ============================
// STANDARD FRAMES (NO 3C0 HERE)
// ============================

struct StdFrame {
  unsigned long id;
  byte len;
  byte data[8];
  unsigned long periodMs;
  unsigned long lastSentMs;
};

// PeriodMs ~= 1240 ms (approx PCM behaviour)
// Payloads are from your boot log (BootSim/boot3).
StdFrame stdFrames[] = {
  {0x575, 4, {0xC7,0x20,0x10,0x00,0x00,0x00,0x00,0x00},       1240, 0},
  {0x386, 8, {0x00,0x00,0x00,0x00,0x50,0x05,0x50,0x00},       1240, 0},
  {0x5F7, 8, {0x80,0xFF,0xFF,0xBF,0xFF,0xFD,0xE7,0xFF},       1240, 0},
  {0x6B2, 8, {0x40,0x0B,0x00,0x20,0x19,0x0E,0x82,0xCF},       1240, 0},
  {0x583, 8, {0x00,0x10,0x00,0x00,0x00,0x44,0x00,0x00},       1240, 0},
  {0x504, 8, {0x00,0x00,0x00,0x00,0xFC,0x77,0xFF,0x3F},       1240, 0},
  {0x3D6, 8, {0x80,0x01,0x00,0x20,0x00,0x00,0x00,0x00},       1240, 0},
  {0x663, 8, {0x08,0x00,0x00,0x0F,0x00,0x01,0x00,0x00},       1240, 0},
  {0x5F5, 8, {0xFF,0x07,0xF8,0xFF,0xFF,0x65,0xE0,0xFF},       1240, 0},

  // Additional standard IDs from boot logs
  {0x2A0, 8, {0x6E,0x01,0x80,0xFC,0xFF,0x0F,0x08,0x00},       1240, 0},
  {0x3E4, 8, {0x00,0xE0,0x1F,0x00,0x00,0x00,0x00,0x00},       1240, 0},
  {0x3FE, 8, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},       1240, 0},
  {0x551, 8, {0x61,0x22,0x0C,0x00,0x00,0x00,0x00,0x00},       1240, 0},
  {0x585, 8, {0x04,0x2C,0x02,0x7F,0x00,0x00,0x00,0x00},       2480, 0},  // bit slower
  {0x590, 8, {0x00,0x00,0x00,0x00,0x00,0xA0,0x3D,0x8C},       2000, 0}
};

const int STD_FRAME_COUNT = sizeof(stdFrames) / sizeof(StdFrame);

// ============================
// 3C0 – COMPRESSED REAL PATTERN
// ============================

struct Timed3C0 {
  uint16_t dt_ms;   // delta time since previous 3C0 frame (ms)
  uint8_t  data[4]; // first 4 bytes of the payload
};

// Approximate compressed sequence extracted from boot3/BootSim behaviour.
// Gives realistic pattern + varied spacing.
const Timed3C0 seq3C0[] = {
  {   0, {0xC0,0x01,0x07,0x00}},
  { 433, {0xDE,0x02,0x23,0x00}},
  { 733, {0x2E,0x01,0x23,0x00}},
  {  16, {0xC0,0x01,0x07,0x00}},
  {  64, {0xC0,0x01,0x07,0x00}},
  { 429, {0xDE,0x02,0x23,0x00}},
  { 710, {0x2E,0x01,0x23,0x00}},
  {  47, {0xC0,0x01,0x07,0x00}},
  {  64, {0xC0,0x01,0x07,0x00}},
  { 440, {0xDE,0x02,0x23,0x00}},
  { 687, {0x2E,0x01,0x23,0x00}},
  {  48, {0xC0,0x01,0x07,0x00}},
  {  64, {0xC0,0x01,0x07,0x00}},
  { 409, {0xDE,0x02,0x23,0x00}},
  { 727, {0x2E,0x01,0x23,0x00}},
  {  38, {0xC0,0x01,0x07,0x00}},
  {  62, {0xC0,0x01,0x07,0x00}},
  { 417, {0xDE,0x02,0x23,0x00}},
  { 712, {0x2E,0x01,0x23,0x00}},
  {  48, {0xC0,0x01,0x07,0x00}},
  {  64, {0xC0,0x01,0x07,0x00}},
  { 430, {0xDE,0x02,0x23,0x00}},
  { 721, {0x2E,0x01,0x23,0x00}},
  {  16, {0xC0,0x01,0x07,0x00}}
};

const uint8_t SEQ3C0_LEN = sizeof(seq3C0) / sizeof(Timed3C0);
uint8_t  seq3C0_index    = 0;
bool     seq3C0_started  = false;
unsigned long last3C0Ms  = 0;

// ============================
// EXTENDED BOOTLOADER FRAMES
// ============================

struct ExtFrame {
  unsigned long id;
  byte len;
  byte data[8];
};

// 0x17331710x group
const ExtFrame group17331710[] = {
  {0x17331710, 3, {0x35,0xCD,0x10,0x00,0x00,0x00,0x00,0x00}},
  {0x17331710, 8, {0x80,0x08,0x35,0xC3,0x38,0x07,0xFF,0x80}},
  {0x17331710, 3, {0x35,0xC4,0x0A,0x00,0x00,0x00,0x00,0x00}},
  {0x17331710, 8, {0x35,0xCE,0xAE,0x02,0x00,0x02,0x00,0x00}}
};

// 0x17332210x group
const ExtFrame group17332210[] = {
  {0x17332210, 3, {0x38,0x84,0x0A,0x00,0x00,0x00,0x00,0x00}},
  {0x17332210, 3, {0x38,0x8F,0x00,0x00,0x00,0x00,0x00,0x00}},
  {0x17332210, 3, {0x38,0x8D,0x10,0x00,0x00,0x00,0x00,0x00}},
  {0x17332210, 8, {0x80,0x08,0x38,0x83,0x38,0x05,0x00,0x00}}
};

// 0x17333910x group
const ExtFrame group17333910[] = {
  {0x17333910, 8, {0x3E,0x53,0xDB,0x00,0x00,0xFE,0xFF,0x00}},
  {0x17333910, 8, {0x3E,0x54,0xDB,0x03,0x00,0xFE,0xFF,0x00}},
  {0x17333910, 8, {0x3E,0x52,0xDB,0x03,0x00,0xFE,0xFF,0x00}},
  {0x17333910, 8, {0x3E,0x55,0xDB,0x03,0x00,0xFE,0xFF,0x00}}
};

// 0x17332310x group
const ExtFrame group17332310[] = {
  {0x17332310, 8, {0x80,0x08,0x38,0xC3,0x38,0x03,0xE0,0x00}},
  {0x17332310, 8, {0x38,0xC2,0x03,0x01,0x23,0x00,0x04,0x03}},
  {0x17332310, 3, {0x38,0xC4,0x00,0x00,0x00,0x00,0x00,0x00}},
  {0x17332310, 3, {0x38,0xCE,0x44,0x00,0x00,0x00,0x00,0x00}}
};

// Singleton extended frames (appear early & in bursts)
const ExtFrame extSingles[] = {
  {0x17F00010, 8, {0x20,0x10,0x00,0x00,0x00,0x00,0x02,0x80}},
  {0x16A95414, 8, {0x00,0xB0,0x08,0x00,0x00,0x00,0x53,0x00}},
  {0x12DD5467, 8, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF8}},
  {0x1B000010, 8, {0x10,0x50,0x04,0x01,0x00,0x00,0x00,0x00}},
  {0x16A955EC, 8, {0x00,0x00,0x00,0x07,0x00,0x00,0x00,0x00}}
};
const int EXT_SINGLES_COUNT = sizeof(extSingles) / sizeof(ExtFrame);

unsigned long lastBurstMs          = 0;
// High intensity: bursts every 200 ms during BOOTLOADER phase
const unsigned long BURST_PERIOD_MS = 200;
byte bootBurstCount                = 0;

// ============================
// HELPERS
// ============================

void sendStd(const StdFrame &f) {
  CAN.sendMsgBuf(f.id, 0, 0, f.len, (byte*)f.data);
}

void sendExt(const ExtFrame &f) {
  CAN.sendMsgBuf(f.id, 1, 0, f.len, (byte*)f.data);
}

// Send one 3C0 frame from compressed sequence
void send3C0(const Timed3C0 &entry) {
  uint8_t buf[8] = {
    entry.data[0],
    entry.data[1],
    entry.data[2],
    entry.data[3],
    0x00, 0x00, 0x00, 0x00
  };
  CAN.sendMsgBuf(0x3C0, 0, 0, 4, buf);
}

// High-intensity bootloader burst
void sendBootloaderBurst() {
  // Singletons in first few bursts only, then rely on grouped traffic
  if (bootBurstCount < 4) {
    for (int i = 0; i < EXT_SINGLES_COUNT; i++) {
      sendExt(extSingles[i]);
      delayMicroseconds(800);
    }
  }

  for (int i = 0; i < (int)(sizeof(group17331710)/sizeof(group17331710[0])); i++) {
    sendExt(group17331710[i]);
    delayMicroseconds(800);
  }
  for (int i = 0; i < (int)(sizeof(group17332210)/sizeof(group17332210[0])); i++) {
    sendExt(group17332210[i]);
    delayMicroseconds(800);
  }
  for (int i = 0; i < (int)(sizeof(group17333910)/sizeof(group17333910[0])); i++) {
    sendExt(group17333910[i]);
    delayMicroseconds(800);
  }
  for (int i = 0; i < (int)(sizeof(group17332310)/sizeof(group17332310[0])); i++) {
    sendExt(group17332310[i]);
    delayMicroseconds(800);
  }

  bootBurstCount++;
}

// ============================
// SETUP
// ============================

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  while (CAN.begin(CAN_500KBPS) != CAN_OK) {
    Serial.println("CAN init FAIL, retrying...");
    delay(100);
  }
  Serial.println("CAN init OK");

  phaseStartMs   = millis();
  last3C0Ms      = millis();
  seq3C0_index   = 0;
  seq3C0_started = false;

  for (int i = 0; i < STD_FRAME_COUNT; i++) {
    stdFrames[i].lastSentMs = millis();
  }
}

// ============================
// MAIN LOOP
// ============================

void loop() {
  unsigned long nowMs   = millis();
  unsigned long elapsed = nowMs - phaseStartMs;

  // ---- Phase selection ----
  if (elapsed < INIT_DURATION_MS) {
    currentPhase = PHASE_INIT;
  } else if (elapsed < INIT_DURATION_MS + BOOTLOADER_DURATION_MS) {
    currentPhase = PHASE_BOOTLOADER;
  } else {
    currentPhase = PHASE_RUNTIME;
    if (elapsed > CYCLE_DURATION_MS) {
      phaseStartMs = nowMs;
      bootBurstCount = 0;
    }
  }

  // ---- 3C0 timing replay (compressed boot-like pattern) ----
  if (!seq3C0_started) {
    send3C0(seq3C0[0]);
    last3C0Ms      = nowMs;
    seq3C0_index   = 1;
    seq3C0_started = true;
  } else {
    const Timed3C0 &entry = seq3C0[seq3C0_index];
    if ((unsigned long)(nowMs - last3C0Ms) >= entry.dt_ms) {
      send3C0(entry);
      last3C0Ms = nowMs;
      seq3C0_index++;
      if (seq3C0_index >= SEQ3C0_LEN) {
        seq3C0_index = 0;   // loop the pattern
      }
    }
  }

  // ---- Send standard frames (all except 3C0) ----
  for (int i = 0; i < STD_FRAME_COUNT; i++) {
    if (nowMs - stdFrames[i].lastSentMs >= stdFrames[i].periodMs) {
      sendStd(stdFrames[i]);
      stdFrames[i].lastSentMs = nowMs;
    }
  }

  // ---- High-intensity bootloader bursts (only in BOOTLOADER phase) ----
  if (currentPhase == PHASE_BOOTLOADER) {
    if (nowMs - lastBurstMs >= BURST_PERIOD_MS) {
      sendBootloaderBurst();
      lastBurstMs = nowMs;
    }
  }

  // No delay(): timing controlled by millis()
}
