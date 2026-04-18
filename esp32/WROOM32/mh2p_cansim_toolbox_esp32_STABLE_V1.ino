/*
 * VAG PCM (MIB / MH2p / PCM5) CAN bus simulator (High Bursts) for ESP32
 * -----------------------------------------------------------
 * - Hardware: ESP32 with built-in TWAI controller + external CAN transceiver
 *             (e.g., SN65HVD230, TJA1050, MCP2562)
 * - Bitrate: 500 kbps
 *
 * Profiles:
 *   - mh2p: original MH2p traffic + boot bursts
 *   - mh2p+cluster: mh2p plus cluster traffic, and cluster 0x3C0 pattern
 *
 * Notes:
 *   - Standard payloads are taken from real system.
 *   - 3C0 cycles through C0/DE/2E patterns with a semi-realistic timing sequence.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "driver/twai.h"

#define CAN_RX_PIN GPIO_NUM_4
#define CAN_TX_PIN GPIO_NUM_5

struct StdFrame {
  unsigned long id;
  byte len;
  byte data[8];
  unsigned long periodMs;
  unsigned long lastSentMs;
};

struct ExtFrame {
  unsigned long id;
  byte len;
  byte data[8];
};

struct ExtPeriodic {
  ExtFrame frame;
  unsigned long periodMs;
  unsigned long lastSentMs;
};

struct Timed3C0 {
  uint16_t dt_ms;
  uint8_t  data[4];
};

static const char* WIFI_SSID = "LocalAP";
static const char* WIFI_PASS = "LocalPASS";
static const char* FALLBACK_AP_SSID = "FallbackAP";
static const char* FALLBACK_AP_PASS = "FallbackPASS";
WebServer server(80);

static bool g_txEnabled = true;
static bool g_kl15On = true;
static bool g_steerEnabled = true;
static bool g_mh2pExtrasEnabled = false;
static const uint8_t KL15_MASK_BITS = 0x03;
static int g_extSinglesIndex = 0;
static const unsigned long MIN_3C0_INTERVAL_MS = 0;
static const unsigned long C3C0_OFFSET_MS = 16;
static const uint32_t TWAI_TX_TIMEOUT_TICKS = 2;

enum ProfileMode { PROFILE_MH2P = 0, PROFILE_CLUSTER = 1 };
static ProfileMode currentProfile = PROFILE_CLUSTER;

// Phase state machine (timing alignment for boot-like traffic)
enum BootPhase {
  PHASE_INIT = 0,
  PHASE_BOOTLOADER,
  PHASE_RUNTIME
};
static BootPhase currentPhase = PHASE_INIT;
static unsigned long phaseStartMs = 0;
static const unsigned long INIT_DURATION_MS = 3000;
static const unsigned long BOOTLOADER_DURATION_MS = 6000;
static const unsigned long CYCLE_DURATION_MS = 20000;

struct IdEntry {
  uint32_t id;
  bool ext;
  const char* label;
  bool enabled;
  uint32_t count;
};
static IdEntry idEntries[] = {
  {0x575, false, "0x575", true, 0},
  {0x386, false, "0x386", true, 0},
  {0x5F7, false, "0x5F7", true, 0},
  {0x6B2, false, "0x6B2", true, 0},
  {0x583, false, "0x583", true, 0},
  {0x504, false, "0x504", true, 0},
  {0x3D6, false, "0x3D6", true, 0},
  {0x663, false, "0x663", true, 0},
  {0x5F5, false, "0x5F5", true, 0},
  {0x2A0, false, "0x2A0", true, 0},
  {0x3E4, false, "0x3E4", true, 0},
  {0x3FE, false, "0x3FE", true, 0},
  {0x551, false, "0x551", true, 0},
  {0x585, false, "0x585", true, 0},
  {0x590, false, "0x590", true, 0},
  {0x3C0, false, "0x3C0", true, 0},
  {0xC, false, "0xC", true, 0},
  {0x10, false, "0x10", true, 0},
  {0x11, false, "0x11", true, 0},
  {0x40, false, "0x40", true, 0},
  {0x48, false, "0x48", true, 0},
  {0x54, false, "0x54", true, 0},
  {0x68, false, "0x68", true, 0},
  {0x80, false, "0x80", true, 0},
  {0x88, false, "0x88", true, 0},
  {0xA8, false, "0xA8", true, 0},
  {0xB0, false, "0xB0", true, 0},
  {0xB8, false, "0xB8", true, 0},
  {0xC8, false, "0xC8", true, 0},
  {0xF0, false, "0xF0", true, 0},
  {0x108, false, "0x108", true, 0},
  {0x128, false, "0x128", true, 0},
  {0x160, false, "0x160", true, 0},
  {0x168, false, "0x168", true, 0},
  {0x1A8, false, "0x1A8", true, 0},
  {0x1D8, false, "0x1D8", true, 0},
  {0x228, false, "0x228", true, 0},
  {0x268, false, "0x268", true, 0},
  {0x278, false, "0x278", true, 0},
  {0x2A8, false, "0x2A8", true, 0},
  {0x2B0, false, "0x2B0", true, 0},
  {0x2E8, false, "0x2E8", true, 0},
  {0x308, false, "0x308", true, 0},
  {0x388, false, "0x388", true, 0},
  {0x448, false, "0x448", true, 0},
  {0x48B, false, "0x48B", true, 0},
  {0x490, false, "0x490", true, 0},
  {0x492, false, "0x492", true, 0},
  {0x4A6, false, "0x4A6", true, 0},
  {0x4A8, false, "0x4A8", true, 0},
  {0x4B1, false, "0x4B1", true, 0},
  {0x4BB, false, "0x4BB", true, 0},
  {0x4C0, false, "0x4C0", true, 0},
  {0x4D4, false, "0x4D4", true, 0},
  {0x4EC, false, "0x4EC", true, 0},
  {0x4FB, false, "0x4FB", true, 0},
  {0x51E, false, "0x51E", true, 0},
  {0x55B, false, "0x55B", true, 0},
  {0x5F0, false, "0x5F0", true, 0},
  {0x5BF, false, "0x5BF", true, 0},
  {0x17331710, true, "0x17331710", true, 0},
  {0x17332210, true, "0x17332210", true, 0},
  {0x17333910, true, "0x17333910", true, 0},
  {0x17332310, true, "0x17332310", true, 0},
  {0x17F00010, true, "0x17F00010", true, 0},
  {0x16A95414, true, "0x16A95414", true, 0},
  {0x12DD5467, true, "0x12DD5467", true, 0},
  {0x1B000010, true, "0x1B000010", true, 0},
  {0x16A955EC, true, "0x16A955EC", true, 0},
};
static const int ID_ENTRY_COUNT = sizeof(idEntries) / sizeof(IdEntry);

static int findIdIndex(uint32_t id, bool ext) { for (int i = 0; i < ID_ENTRY_COUNT; i++) { if (idEntries[i].id == id && idEntries[i].ext == ext) return i; } return -1; }
static bool isEnabled(uint32_t id, bool ext) { int idx = findIdIndex(id, ext); if (idx < 0) return true; return idEntries[idx].enabled; }
static void noteSent(uint32_t id, bool ext) { int idx = findIdIndex(id, ext); if (idx < 0) return; idEntries[idx].count++; }
static void redirectHome() { server.sendHeader("Location", "/"); server.send(302, "text/plain", ""); }
// MH2P STANDARD FRAMES
StdFrame stdFrames_mh2p[] = {
  {0x575, 4, {0xC7,0x20,0x10,0x00,0x00,0x00,0x00,0x00}, 620, 0},
  {0x386, 8, {0x00,0x00,0x00,0x00,0x50,0x05,0x50,0x00}, 1240, 0},
  {0x5f7, 8, {0x80,0xFF,0xFF,0xBF,0xFF,0xFD,0xE7,0xFF}, 1240, 0},
  {0x6b2, 8, {0x40,0x0B,0x00,0x20,0x19,0x0E,0x82,0xCF}, 1240, 0},
  {0x583, 8, {0x00,0x10,0x00,0x00,0x00,0x44,0x00,0x00}, 1240, 0},
  {0x504, 8, {0x00,0x00,0x00,0x00,0xFC,0x77,0xFF,0x3F}, 1240, 0},
  {0x3d6, 8, {0x80,0x01,0x00,0x20,0x00,0x00,0x00,0x00}, 1240, 0},
  {0x663, 8, {0x08,0x00,0x00,0x0F,0x00,0x01,0x00,0x00}, 1240, 0},
  {0x5f5, 8, {0xFF,0x07,0xF8,0xFF,0xFF,0x65,0xE0,0xFF}, 1240, 0},
  {0x2a0, 8, {0x6E,0x01,0x80,0xFC,0xFF,0x0F,0x08,0x00}, 1240, 0},
  {0x3e4, 8, {0x00,0xE0,0x1F,0x00,0x00,0x00,0x00,0x00}, 1240, 0},
  {0x3fe, 8, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, 1240, 0},
  {0x551, 8, {0x61,0x22,0x0C,0x00,0x00,0x00,0x00,0x00}, 1240, 0},
  {0x585, 8, {0x04,0x2C,0x02,0x7F,0x00,0x00,0x00,0x00}, 1240, 0},
  {0x590, 8, {0x00,0x00,0x00,0x00,0x00,0xA0,0x3D,0x8C}, 1240, 0},
};
const int STD_FRAME_MH2P_COUNT = sizeof(stdFrames_mh2p) / sizeof(StdFrame);

// CLUSTER STANDARD FRAMES
const StdFrame clusterStdFramesConst[] = {
  {0xc, 2, {0x00,0x08,0x00,0x00,0x00,0x00,0x00,0x00}, 361, 0},
  {0x10, 8, {0x00,0x76,0xF8,0x5F,0x00,0x00,0x1E,0x07}, 8, 0},
  {0x11, 8, {0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, 361, 0},
  {0x40, 8, {0x68,0x02,0x6E,0x00,0x00,0x9F,0x01,0xF6}, 361, 0},
  {0x48, 8, {0xFF,0x80,0x00,0x00,0x40,0x77,0x03,0xFF}, 361, 0},
  {0x54, 3, {0x00,0x80,0x00,0x00,0x00,0x00,0x00,0x00}, 361, 0},
  {0x68, 8, {0x7C,0x00,0x40,0x39,0x1D,0xD7,0x35,0x24}, 361, 0},
  {0x80, 8, {0x00,0x04,0x38,0x64,0x00,0x05,0x30,0x00}, 361, 0},
  {0x88, 8, {0xFF,0x0B,0x07,0x18,0x48,0x00,0x00,0x00}, 361, 0},
  {0xa8, 8, {0xFE,0xF0,0x11,0x09,0xBF,0x38,0x00,0x00}, 361, 0},
  {0xb0, 4, {0x00,0x00,0x1C,0x22,0x00,0x00,0x00,0x00}, 361, 0},
  {0xb8, 8, {0xA8,0xE6,0xF9,0x00,0x00,0x00,0x05,0xFC}, 361, 0},
  {0xc8, 8, {0x83,0x0C,0x4F,0x95,0x1E,0x39,0x7C,0x3C}, 361, 0},
  {0xf0, 8, {0x40,0x01,0x82,0x00,0x84,0x00,0x04,0x38}, 361, 0},
  {0x108, 8, {0xDD,0x00,0x63,0xE5,0x01,0x80,0x29,0xF4}, 361, 0},
  {0x128, 8, {0xC0,0x00,0x00,0x00,0x00,0x00,0x3F,0x3F}, 9, 0},
  {0x160, 8, {0x04,0xAC,0xA3,0x57,0x08,0x6D,0x86,0x00}, 361, 0},
  {0x168, 8, {0x98,0x08,0x15,0x64,0x81,0x00,0x84,0x59}, 361, 0},
  {0x1a8, 8, {0x00,0x00,0x3C,0x78,0x22,0x7F,0x01,0xDA}, 361, 0},
  {0x1d8, 8, {0x10,0x00,0x00,0x00,0x06,0xB2,0xC5,0x98}, 361, 0},
  {0x228, 8, {0x10,0xE5,0x00,0x80,0x00,0x00,0x2B,0x7F}, 361, 0},
  {0x268, 8, {0x2B,0x08,0x00,0x00,0x44,0x02,0x19,0x02}, 361, 0},
  {0x278, 8, {0x7F,0x00,0x00,0x80,0x00,0x00,0x00,0x00}, 361, 0},
  {0x2a8, 8, {0xD8,0x00,0x0F,0x48,0x21,0x69,0x00,0x00}, 361, 0},
  {0x2b0, 8, {0x20,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, 361, 0},
  {0x2e8, 8, {0x98,0x00,0x00,0x00,0x41,0x00,0x2B,0x7F}, 361, 0},
  {0x308, 8, {0xF0,0x00,0x40,0x00,0x00,0x05,0x00,0x3F}, 361, 0},
  {0x388, 8, {0xD6,0x89,0xC8,0x01,0xB3,0x00,0x2B,0x7F}, 361, 0},
  {0x448, 8, {0x00,0x18,0x13,0x00,0x00,0x00,0x00,0x00}, 361, 0},
  {0x48b, 8, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, 361, 0},
  {0x490, 8, {0x00,0x80,0x05,0xF5,0xEB,0x00,0x00,0xFF}, 361, 0},
  {0x492, 8, {0x00,0x00,0x05,0x20,0x00,0x68,0xD1,0x00}, 361, 0},
  {0x4a6, 8, {0x08,0x00,0x00,0x00,0x00,0x00,0x01,0x60}, 361, 0},
  {0x4a8, 8, {0x01,0x41,0x00,0x00,0x00,0x00,0xEA,0x00}, 361, 0},
  {0x4b1, 8, {0x00,0x10,0x00,0x00,0x00,0x05,0x5F,0x00}, 361, 0},
  {0x4bb, 8, {0xF0,0x00,0x40,0xC0,0x0F,0xFF,0x00,0x00}, 361, 0},
  {0x4c0, 8, {0x6F,0x00,0x10,0xFC,0x00,0x00,0x00,0x00}, 361, 0},
  {0x4d4, 8, {0x00,0xE3,0x00,0x00,0x00,0x00,0x0C,0xA9}, 361, 0},
  {0x4ec, 8, {0x00,0x00,0xFF,0xFF,0xFF,0xFF,0x00,0xFF}, 361, 0},
  {0x4fb, 8, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, 361, 0},
  {0x51e, 8, {0x58,0x02,0x00,0x00,0x11,0x00,0x00,0x00}, 361, 0},
  {0x55b, 8, {0x5B,0x01,0x00,0x02,0x5D,0x05,0x00,0xE0}, 361, 0},
  {0x5f0, 8, {0xFF,0xFF,0x7F,0x00,0x00,0x7F,0x00,0x00}, 361, 0},
};
const int STD_FRAME_CLUSTER_COUNT = sizeof(clusterStdFramesConst) / sizeof(StdFrame);
StdFrame stdFrames_cluster[STD_FRAME_CLUSTER_COUNT];

// 3C0 SEQUENCES
const Timed3C0 seq3C0_mh2p[] = {
  {0, {0xC0,0x01,0x07,0x00}},
  {433, {0xDE,0x02,0x23,0x00}},
  {733, {0x2E,0x01,0x23,0x00}},
  {16, {0xC0,0x01,0x07,0x00}},
  {64, {0xC0,0x01,0x07,0x00}},
  {429, {0xDE,0x02,0x23,0x00}},
  {710, {0x2E,0x01,0x23,0x00}},
  {47, {0xC0,0x01,0x07,0x00}},
  {64, {0xC0,0x01,0x07,0x00}},
  {440, {0xDE,0x02,0x23,0x00}},
  {687, {0x2E,0x01,0x23,0x00}},
  {48, {0xC0,0x01,0x07,0x00}},
  {64, {0xC0,0x01,0x07,0x00}},
  {409, {0xDE,0x02,0x23,0x00}},
  {727, {0x2E,0x01,0x23,0x00}},
  {38, {0xC0,0x01,0x07,0x00}},
  {62, {0xC0,0x01,0x07,0x00}},
  {417, {0xDE,0x02,0x23,0x00}},
  {712, {0x2E,0x01,0x23,0x00}},
  {48, {0xC0,0x01,0x07,0x00}},
  {64, {0xC0,0x01,0x07,0x00}},
  {430, {0xDE,0x02,0x23,0x00}},
  {721, {0x2E,0x01,0x23,0x00}},
  {16, {0xC0,0x01,0x07,0x00}},
};
const uint8_t SEQ3C0_MH2P_LEN = sizeof(seq3C0_mh2p) / sizeof(Timed3C0);
uint8_t seq3C0_index_mh2p = 0;
bool seq3C0_started_mh2p = false;
unsigned long last3C0Ms_mh2p = 0;

const Timed3C0 seq3C0_cluster[] = {
  {0, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {352, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {354, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {352, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {352, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {354, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {354, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {354, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {352, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {352, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {354, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {354, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {352, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {352, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {354, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {354, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {352, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {894, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {352, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {354, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {354, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {352, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {354, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {352, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {352, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {354, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {354, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {354, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {354, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {352, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {894, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {354, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {9, {0xC0,0x01,0x07,0x00}},
  {352, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {7, {0xC0,0x01,0x07,0x00}},
  {354, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {352, {0x15,0x09,0x23,0x00}},
  {8, {0xC0,0x01,0x07,0x00}},
  {353, {0x15,0x09,0x23,0x00}},
  {6, {0xC0,0x01,0x07,0x00}},
};
const uint16_t SEQ3C0_CLUSTER_LEN = sizeof(seq3C0_cluster) / sizeof(Timed3C0);
uint16_t seq3C0_index_cluster = 0;
bool seq3C0_started_cluster = false;
unsigned long last3C0Ms_cluster = 0;

// EXTENDED BOOTLOADER FRAMES
const ExtFrame group17331710[] = {
  {0x17331710, 3, {0x35,0xCD,0x10,0x00,0x00,0x00,0x00,0x00}},
  {0x17331710, 8, {0x80,0x08,0x35,0xC3,0x38,0x07,0xFF,0x80}},
  {0x17331710, 3, {0x35,0xC4,0x0A,0x00,0x00,0x00,0x00,0x00}},
  {0x17331710, 8, {0x35,0xCE,0xAE,0x02,0x00,0x02,0x00,0x00}}
};

const ExtFrame group17332210[] = {
  {0x17332210, 3, {0x38,0x84,0x0A,0x00,0x00,0x00,0x00,0x00}},
  {0x17332210, 3, {0x38,0x8F,0x00,0x00,0x00,0x00,0x00,0x00}},
  {0x17332210, 3, {0x38,0x8D,0x10,0x00,0x00,0x00,0x00,0x00}},
  {0x17332210, 8, {0x80,0x08,0x38,0x83,0x38,0x05,0x00,0x00}}
};

const ExtFrame group17333910[] = {
  {0x17333910, 8, {0x3E,0x53,0xDB,0x00,0x00,0xFE,0xFF,0x00}},
  {0x17333910, 8, {0x3E,0x54,0xDB,0x03,0x00,0xFE,0xFF,0x00}},
  {0x17333910, 8, {0x3E,0x52,0xDB,0x03,0x00,0xFE,0xFF,0x00}},
  {0x17333910, 8, {0x3E,0x55,0xDB,0x03,0x00,0xFE,0xFF,0x00}}
};

const ExtFrame group17332310[] = {
  {0x17332310, 8, {0x80,0x08,0x38,0xC3,0x38,0x03,0xE0,0x00}},
  {0x17332310, 8, {0x38,0xC2,0x03,0x01,0x23,0x00,0x04,0x03}},
  {0x17332310, 3, {0x38,0xC4,0x00,0x00,0x00,0x00,0x00,0x00}},
  {0x17332310, 3, {0x38,0xCE,0x44,0x00,0x00,0x00,0x00,0x00}}
};

const unsigned long EXT_SINGLE_PERIOD_MS = 1240;
ExtPeriodic extSingles[] = {
  {{0x17F00010, 8, {0x20,0x10,0x00,0x00,0x00,0x00,0x02,0x80}}, EXT_SINGLE_PERIOD_MS, 0},
  {{0x16A95414, 8, {0x00,0xB0,0x08,0x00,0x00,0x00,0x53,0x00}}, EXT_SINGLE_PERIOD_MS, 0},
  {{0x12DD5467, 8, {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xF8}}, EXT_SINGLE_PERIOD_MS, 0},
  {{0x1B000010, 8, {0x10,0x50,0x04,0x01,0x00,0x00,0x00,0x00}}, EXT_SINGLE_PERIOD_MS, 0},
  {{0x16A955EC, 8, {0x00,0x00,0x00,0x07,0x00,0x00,0x00,0x00}}, EXT_SINGLE_PERIOD_MS, 0}
};
const int EXT_SINGLES_COUNT = sizeof(extSingles) / sizeof(ExtPeriodic);

unsigned long lastBurstMs = 0;
const unsigned long BURST_PERIOD_MS = 1237;
const unsigned long BURST_INTERFRAME_DELAY_MS = 30;

const ExtFrame* burstSeq[] = {
  &group17331710[0], &group17331710[1], &group17331710[2], &group17331710[3],
  &group17332210[0], &group17332210[1], &group17332210[2], &group17332210[3],
  &group17333910[0], &group17333910[1], &group17333910[2], &group17333910[3],
  &group17332310[0], &group17332310[1], &group17332310[2], &group17332310[3]
};
const int BURST_SEQ_COUNT = sizeof(burstSeq) / sizeof(burstSeq[0]);
static bool g_burstActive = false;
static int g_burstIndex = 0;
static unsigned long g_burstNextMs = 0;

// STEERING WHEEL INJECTOR
static bool driver_installed = false;
uint16_t steer_curId = 0x5BF;
uint16_t steer_basePeriodMs = 20;
uint16_t steer_pulseMs = 80;
uint16_t steer_longHoldMs = 800;
uint16_t steer_longRepMs = 1019;
uint8_t steer_basePayload[8] = {0x00,0x00,0x00,0x31,0x00,0x00,0x00,0x00};
uint8_t steer_eventB0 = 0x00;
uint8_t steer_eventB1 = 0x00;
uint8_t steer_eventB2 = 0x00;
uint8_t steer_eventB3 = 0x31;
enum CntMode { CNT_OFF=0, CNT_LOW_NIBBLE=1, CNT_HIGH_NIBBLE=2, CNT_BOTH=3 };
CntMode steer_cntMode = CNT_OFF;
uint8_t steer_cntNibble = 0;
uint8_t steer_cntByteIndex = 1;
enum CrcMode { CRC_OFF=0, CRC_XOR=1, CRC_SUM=2 };
CrcMode steer_crcMode = CRC_OFF;
uint8_t steer_crcByteIndex = 0;

static inline bool steer_isHexChar(char c) { return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'); }
uint16_t steer_parseHex16(const String &s) {
  int start = 0; if (s.startsWith("0x") || s.startsWith("0X")) start = 2; uint16_t v = 0;
  for (int i = start; i < (int)s.length(); i++) { char c = s[i]; if (!steer_isHexChar(c)) break; v <<= 4; if (c >= '0' && c <= '9') v |= (c - '0'); else if (c >= 'a' && c <= 'f') v |= (c - 'a' + 10); else v |= (c - 'A' + 10); }
  return v; }
uint8_t steer_parseHex8(const String &s) { return (uint8_t)(steer_parseHex16(s) & 0xFF); }
long steer_parseDec(const String &s) { return s.toInt(); }
uint8_t steer_computeXorCrc(const uint8_t d[8], uint8_t skipIdx) { uint8_t x = 0; for (int i = 0; i < 8; i++) if (i != skipIdx) x ^= d[i]; return x; }
uint8_t steer_computeSumCrc(const uint8_t d[8], uint8_t skipIdx) { uint16_t s = 0; for (int i = 0; i < 8; i++) if (i != skipIdx) s += d[i]; return (uint8_t)(s & 0xFF); }
void steer_applyCounter(uint8_t d[8]) { if (steer_cntMode == CNT_OFF) return; uint8_t v = (steer_cntNibble & 0x0F); uint8_t b = d[steer_cntByteIndex]; switch (steer_cntMode) { case CNT_LOW_NIBBLE: b = (b & 0xF0) | v; break; case CNT_HIGH_NIBBLE: b = (b & 0x0F) | (v << 4); break; case CNT_BOTH: b = (v << 4) | v; break; default: break; } d[steer_cntByteIndex] = b; }
void steer_sendFrame(uint16_t id, const uint8_t data[8]) { if (!g_txEnabled || !g_steerEnabled) return; if (!isEnabled(id, false)) return; twai_message_t msg = {}; msg.identifier = id; msg.data_length_code = 8; msg.flags = 0; for (int i = 0; i < 8; i++) msg.data[i] = data[i]; twai_transmit(&msg, TWAI_TX_TIMEOUT_TICKS); noteSent(id, false); }
void steer_buildPayload(uint8_t out[8], bool overlayEvent) { for (int i = 0; i < 8; i++) out[i] = steer_basePayload[i]; if (overlayEvent) { out[0] = steer_eventB0; out[1] = steer_eventB1; out[2] = steer_eventB2; out[3] = steer_eventB3; } steer_applyCounter(out); if (steer_crcMode != CRC_OFF) { uint8_t crc = (steer_crcMode == CRC_XOR) ? steer_computeXorCrc(out, steer_crcByteIndex) : steer_computeSumCrc(out, steer_crcByteIndex); out[steer_crcByteIndex] = crc; } }
void steer_sendIdleOrEvent(bool overlayEvent) { uint8_t buf[8]; steer_buildPayload(buf, overlayEvent); steer_sendFrame(steer_curId, buf); steer_cntNibble = (uint8_t)((steer_cntNibble + 1) & 0x0F); }
enum MacroType { MACRO_NONE=0, MACRO_TAP, MACRO_LONG, MACRO_KNOB };
MacroType steer_macroType = MACRO_NONE;
unsigned long steer_macroStartMs = 0;
unsigned long steer_macroNextMs = 0;
int steer_knobRemaining = 0;
bool steer_knobUp = true;
bool steer_knobFast = false;
bool steer_macroActive() { return steer_macroType != MACRO_NONE; }
void steer_macroStop(bool sendIdleNow=true) { steer_macroType = MACRO_NONE; if (sendIdleNow) { steer_eventB2 = 0x00; steer_sendIdleOrEvent(false); } }
uint8_t steer_btnCodeFromName(const String &name) { if (name == "menu") return 0x01; if (name == "right") return 0x02; if (name == "left") return 0x03; if (name == "ok") return 0x08; if (name == "voice") return 0x19; if (name == "phone") return 0x1C; if (name == "mute") return 0x20; if (name == "view") return 0x23; if (name == "next") return 0x15; if (name == "prev") return 0x16; return 0x00; }
void steer_macroTap(uint8_t b0) { steer_macroStop(false); steer_eventB0 = b0; steer_eventB1 = 0x00; steer_eventB3 = 0x31; steer_macroType = MACRO_TAP; steer_macroStartMs = millis(); steer_macroNextMs = steer_macroStartMs; steer_eventB2 = 0x01; steer_sendIdleOrEvent(true); }
void steer_macroLongStart(uint8_t b0) { steer_macroStop(false); steer_eventB0 = b0; steer_eventB1 = 0x00; steer_eventB3 = 0x31; steer_macroType = MACRO_LONG; steer_macroStartMs = millis(); steer_eventB2 = 0x01; steer_sendIdleOrEvent(true); steer_macroNextMs = steer_macroStartMs + steer_longHoldMs; }
void steer_macroKnobBurst(bool up, bool fast) { steer_macroStop(false); steer_eventB0 = 0x06; steer_eventB1 = 0x00; steer_eventB3 = 0x31; steer_macroType = MACRO_KNOB; steer_macroStartMs = millis(); steer_macroNextMs = steer_macroStartMs; steer_eventB2 = up ? (fast ? 0x02 : 0x01) : 0x0F; steer_sendIdleOrEvent(true); steer_macroNextMs = millis() + 30; }
void steer_macroKnobStart(bool up, bool fast) { steer_knobRemaining = fast ? 4 : 6; steer_knobUp = up; steer_knobFast = fast; steer_macroKnobBurst(up, fast); }
void steer_macroTick(unsigned long now) {
  if (!steer_macroActive()) return; if (now < steer_macroNextMs) return;
  if (steer_macroType == MACRO_TAP) {
    if (now - steer_macroStartMs >= steer_pulseMs) { steer_macroStop(true); }
    else { steer_sendIdleOrEvent(true); steer_macroNextMs = now + 20; }
    return; }
  if (steer_macroType == MACRO_LONG) {
    unsigned long elapsed = now - steer_macroStartMs;
    if (elapsed >= steer_longHoldMs && steer_eventB2 == 0x01) { steer_eventB2 = 0x04; steer_sendIdleOrEvent(true); steer_macroNextMs = now + 200; return; }
    if (steer_eventB2 == 0x04) { steer_eventB2 = 0x05; steer_sendIdleOrEvent(true); steer_macroNextMs = now + 200; return; }
    if (steer_eventB2 == 0x05) { steer_eventB2 = 0x06; steer_sendIdleOrEvent(true); steer_macroNextMs = now + steer_longRepMs; return; }
    if (steer_eventB2 == 0x06) { steer_sendIdleOrEvent(true); steer_macroNextMs = now + steer_longRepMs; return; }
    steer_sendIdleOrEvent(true); steer_macroNextMs = now + 200; return; }
  if (steer_macroType == MACRO_KNOB) {
    if (steer_knobRemaining <= 0) { steer_macroStop(true); return; }
    static bool knobSendIdleNext = false;
    if (!knobSendIdleNext) { steer_sendIdleOrEvent(true); knobSendIdleNext = true; steer_macroNextMs = now + 30; }
    else { steer_sendIdleOrEvent(false); knobSendIdleNext = false; steer_knobRemaining--; steer_macroNextMs = now + 30; }
    return; }
}

bool steer_sweepEnabled = false;
unsigned long steer_sweepNextMs = 0;
uint16_t steer_sweepStepMs = 900;
const uint8_t steer_sweepBtnCodes[] = {0x01,0x02,0x03,0x06,0x08,0x15,0x16,0x19,0x1C,0x20,0x23};
const int STEER_SWEEP_BTN_COUNT = sizeof(steer_sweepBtnCodes)/sizeof(steer_sweepBtnCodes[0]);
int steer_sweepBtnIndex = 0;
const uint8_t steer_sweepActCodes[] = {0x01,0x04,0x05,0x06,0x0F,0x02};
const int STEER_SWEEP_ACT_COUNT = sizeof(steer_sweepActCodes)/sizeof(steer_sweepActCodes[0]);
int steer_sweepActIndex = 0;
int steer_sweepCntMode = 0;
int steer_sweepCrcMode = 0;
void steer_sweepStep() {
  steer_cntMode = (CntMode)steer_sweepCntMode; steer_crcMode = (CrcMode)steer_sweepCrcMode;
  steer_eventB0 = steer_sweepBtnCodes[steer_sweepBtnIndex]; steer_eventB1 = 0x00; steer_eventB2 = steer_sweepActCodes[steer_sweepActIndex]; steer_eventB3 = 0x31;
  steer_sendIdleOrEvent(true);
  steer_sweepCrcMode++; if (steer_sweepCrcMode >= 3) { steer_sweepCrcMode = 0; steer_sweepCntMode++; }
  if (steer_sweepCntMode >= 4) { steer_sweepCntMode = 0; steer_sweepActIndex++; if (steer_sweepActIndex >= STEER_SWEEP_ACT_COUNT) { steer_sweepActIndex = 0; steer_sweepBtnIndex++; } if (steer_sweepBtnIndex >= STEER_SWEEP_BTN_COUNT) steer_sweepBtnIndex = 0; }
}

String steer_stateString() {
  String out; out.reserve(512);
  out += "ID=0x" + String(steer_curId, HEX);
  out += " period=" + String(steer_basePeriodMs);
  out += " pulse=" + String(steer_pulseMs);
  out += " holdms=" + String(steer_longHoldMs);
  out += " repms=" + String(steer_longRepMs) + "\n";
  out += "base=";
  for (int i = 0; i < 8; i++) { if (steer_basePayload[i] < 16) out += "0"; out += String(steer_basePayload[i], HEX); }
  out += "\n";
  out += "event=" + String(steer_eventB0, HEX) + " " + String(steer_eventB1, HEX) + " " + String(steer_eventB2, HEX) + " " + String(steer_eventB3, HEX) + " ...\n";
  out += "cntMode=" + String((int)steer_cntMode) + " cntByte=" + String(steer_cntByteIndex) + " crcMode=" + String((int)steer_crcMode) + " crcByte=" + String(steer_crcByteIndex) + "\n";
  out += "macro=" + String((int)steer_macroType) + " sweep=" + String(steer_sweepEnabled ? "on" : "off") + " sweepms=" + String(steer_sweepStepMs) + "\n";
  return out; }
void steer_setBaseFromHex16(const String &hex16) { for (int i = 0; i < 8; i++) { String byteStr = "0x"; byteStr += hex16[i*2]; byteStr += hex16[i*2 + 1]; steer_basePayload[i] = steer_parseHex8(byteStr); } }
String steer_handleCommand(String cmdLine) {
  String s = cmdLine; s.trim(); if (!s.length()) return ""; int sp1 = s.indexOf(' '); String cmd = (sp1 < 0) ? s : s.substring(0, sp1); cmd.toLowerCase();
  auto rest = [&]() -> String { if (sp1 < 0) return ""; String a = s.substring(sp1 + 1); a.trim(); return a; };
  auto arg1 = [&]() -> String { String a = rest(); int sp = a.indexOf(' '); return (sp < 0) ? a : a.substring(0, sp); };
  if (cmd == "help") { return "Commands:\n  help | state\n  id <hex>\n  base <16hex>\n  period <ms> | pulse <ms> | holdms <ms> | repms <ms>\n  tap <name>   menu,left,right,ok,view,next,prev,mute,phone,voice\n  long <name>\n  release\n  knob up|down|up2|down2\n  b0 <hex> | b1 <hex> | b2 <hex> | b3 <hex>\n  send\n  cnt off|low|high|both | cntbyte <0-7>\n  crc off|xor|sum | crcbyte <0-7>\n  sweep start|stop | sweepms <ms>\n"; }
  if (cmd == "state") return steer_stateString();
  if (cmd == "id") { steer_curId = steer_parseHex16(arg1()); return "OK id=0x" + String(steer_curId, HEX) + "\n"; }
  if (cmd == "base") { String a = arg1(); if (a.length() != 16) return "ERR base needs 16 hex chars\n"; steer_setBaseFromHex16(a); return "OK base set\n"; }
  if (cmd == "period") { steer_basePeriodMs = (uint16_t)steer_parseDec(arg1()); return "OK period set\n"; }
  if (cmd == "pulse")  { steer_pulseMs = (uint16_t)steer_parseDec(arg1()); return "OK pulse set\n"; }
  if (cmd == "holdms") { steer_longHoldMs = (uint16_t)steer_parseDec(arg1()); return "OK holdms set\n"; }
  if (cmd == "repms")  { steer_longRepMs = (uint16_t)steer_parseDec(arg1()); return "OK repms set\n"; }
  if (cmd == "tap") { String a = arg1(); a.toLowerCase(); uint8_t code = steer_btnCodeFromName(a); if (!code) return "ERR unknown tap name\n"; steer_macroTap(code); return "OK tap started\n"; }
  if (cmd == "long") { String a = arg1(); a.toLowerCase(); uint8_t code = steer_btnCodeFromName(a); if (!code) return "ERR unknown long name\n"; steer_macroLongStart(code); return "OK long started (use release)\n"; }
  if (cmd == "release") { steer_macroStop(true); return "OK released\n"; }
  if (cmd == "knob") { String a = arg1(); a.toLowerCase(); if (a == "up") { steer_macroKnobStart(true, false); return "OK knob up\n"; } if (a == "down") { steer_macroKnobStart(false, false); return "OK knob down\n"; } if (a == "up2") { steer_macroKnobStart(true, true); return "OK knob up2\n"; } if (a == "down2") { steer_macroKnobStart(false, true); return "OK knob down2\n"; } return "ERR knob up|down|up2|down2\n"; }
  if (cmd == "b0") { steer_eventB0 = steer_parseHex8(arg1()); return "OK b0 set\n"; }
  if (cmd == "b1") { steer_eventB1 = steer_parseHex8(arg1()); return "OK b1 set\n"; }
  if (cmd == "b2") { steer_eventB2 = steer_parseHex8(arg1()); return "OK b2 set\n"; }
  if (cmd == "b3") { steer_eventB3 = steer_parseHex8(arg1()); return "OK b3 set\n"; }
  if (cmd == "send") { steer_sendIdleOrEvent(true); return "OK sent one event frame\n"; }
  if (cmd == "cnt") { String a = arg1(); a.toLowerCase(); if (a == "off") steer_cntMode = CNT_OFF; else if (a == "low") steer_cntMode = CNT_LOW_NIBBLE; else if (a == "high") steer_cntMode = CNT_HIGH_NIBBLE; else if (a == "both") steer_cntMode = CNT_BOTH; else return "ERR cnt off|low|high|both\n"; return "OK cnt set\n"; }
  if (cmd == "cntbyte") { int v = (int)steer_parseDec(arg1()); if (v < 0 || v > 7) return "ERR cntbyte 0..7\n"; steer_cntByteIndex = (uint8_t)v; return "OK cntbyte set\n"; }
  if (cmd == "crc") { String a = arg1(); a.toLowerCase(); if (a == "off") steer_crcMode = CRC_OFF; else if (a == "xor") steer_crcMode = CRC_XOR; else if (a == "sum") steer_crcMode = CRC_SUM; else return "ERR crc off|xor|sum\n"; return "OK crc set\n"; }
  if (cmd == "crcbyte") { int v = (int)steer_parseDec(arg1()); if (v < 0 || v > 7) return "ERR crcbyte 0..7\n"; steer_crcByteIndex = (uint8_t)v; return "OK crcbyte set\n"; }
  if (cmd == "sweep") { String a = arg1(); a.toLowerCase(); if (a == "start") { steer_sweepEnabled = true; steer_sweepNextMs = millis() + 50; return "OK sweep on\n"; } if (a == "stop") { steer_sweepEnabled = false; return "OK sweep off\n"; } return "ERR sweep start|stop\n"; }
  if (cmd == "sweepms") { int v = (int)steer_parseDec(arg1()); if (v < 100 || v > 10000) return "ERR sweepms 100..10000\n"; steer_sweepStepMs = (uint16_t)v; return "OK sweepms set\n"; }
  return "ERR Unknown cmd\n"; }
unsigned long steer_lastIdleMs = 0;
String steer_line;
void steer_tick(unsigned long now) { if (!driver_installed || !g_steerEnabled) return; if (now - steer_lastIdleMs >= steer_basePeriodMs) { steer_sendIdleOrEvent(steer_macroActive()); steer_lastIdleMs = now; } steer_macroTick(now); if (steer_sweepEnabled && now >= steer_sweepNextMs) { if (!steer_macroActive()) steer_sweepStep(); steer_sweepNextMs = now + steer_sweepStepMs; } }


// ----------------------
// CORE SEND HELPERS
// ----------------------

void sendStd(const StdFrame &f) {
  if (!g_txEnabled) return;
  if (!isEnabled(f.id, false)) return;
  twai_message_t msg = {};
  msg.identifier = f.id;
  msg.extd = 0;
  msg.rtr = 0;
  msg.data_length_code = f.len;
  for (int i = 0; i < f.len; i++) {
    msg.data[i] = f.data[i];
  }
  if (twai_transmit(&msg, TWAI_TX_TIMEOUT_TICKS) == ESP_OK) {
    noteSent(f.id, false);
  }
}

void sendExt(const ExtFrame &f) {
  if (!g_txEnabled) return;
  if (!isEnabled(f.id, true)) return;
  twai_message_t msg = {};
  msg.identifier = f.id;
  msg.extd = 1;
  msg.rtr = 0;
  msg.data_length_code = f.len;
  for (int i = 0; i < f.len; i++) {
    msg.data[i] = f.data[i];
  }
  if (twai_transmit(&msg, TWAI_TX_TIMEOUT_TICKS) == ESP_OK) {
    noteSent(f.id, true);
  }
}

void send3C0Payload(const uint8_t data4[4]) {
  if (!g_txEnabled) return;
  if (!isEnabled(0x3C0, false)) return;
  uint8_t buf[8] = { data4[0], data4[1], data4[2], data4[3], 0x00, 0x00, 0x00, 0x00 };
  if (!g_kl15On) {
    buf[2] &= ~KL15_MASK_BITS;
  }
  twai_message_t msg = {};
  msg.identifier = 0x3C0;
  msg.extd = 0;
  msg.rtr = 0;
  msg.data_length_code = 4;
  for (int i = 0; i < 4; i++) {
    msg.data[i] = buf[i];
  }
  if (twai_transmit(&msg, TWAI_TX_TIMEOUT_TICKS) == ESP_OK) {
    noteSent(0x3C0, false);
  }
}

void tick3C0_mh2p(unsigned long nowMs) {
  if (!seq3C0_started_mh2p) {
    send3C0Payload(seq3C0_mh2p[0].data);
    last3C0Ms_mh2p = nowMs;
    seq3C0_index_mh2p = 1;
    seq3C0_started_mh2p = true;
    return;
  }
  const Timed3C0 &entry = seq3C0_mh2p[seq3C0_index_mh2p];
  unsigned long minWait = entry.dt_ms + C3C0_OFFSET_MS;
  if (minWait < MIN_3C0_INTERVAL_MS) minWait = MIN_3C0_INTERVAL_MS;
  if ((unsigned long)(nowMs - last3C0Ms_mh2p) >= minWait) {
    send3C0Payload(entry.data);
    last3C0Ms_mh2p = nowMs;
    seq3C0_index_mh2p++;
    if (seq3C0_index_mh2p >= SEQ3C0_MH2P_LEN) seq3C0_index_mh2p = 0;
  }
}

void tick3C0_cluster(unsigned long nowMs) {
  if (SEQ3C0_CLUSTER_LEN == 0) return;
  if (!seq3C0_started_cluster) {
    send3C0Payload(seq3C0_cluster[0].data);
    last3C0Ms_cluster = nowMs;
    seq3C0_index_cluster = 1;
    seq3C0_started_cluster = true;
    return;
  }
  const Timed3C0 &entry = seq3C0_cluster[seq3C0_index_cluster];
  unsigned long minWait = entry.dt_ms + C3C0_OFFSET_MS;
  if (minWait < MIN_3C0_INTERVAL_MS) minWait = MIN_3C0_INTERVAL_MS;
  if ((unsigned long)(nowMs - last3C0Ms_cluster) >= minWait) {
    send3C0Payload(entry.data);
    last3C0Ms_cluster = nowMs;
    seq3C0_index_cluster++;
    if (seq3C0_index_cluster >= SEQ3C0_CLUSTER_LEN) seq3C0_index_cluster = 0;
  }
}

void sendExtPeriodic() {
  unsigned long now = millis();
  for (int n = 0; n < EXT_SINGLES_COUNT; n++) {
    int i = (g_extSinglesIndex + n) % EXT_SINGLES_COUNT;
    if (now - extSingles[i].lastSentMs >= extSingles[i].periodMs) {
      sendExt(extSingles[i].frame);
      extSingles[i].lastSentMs = now;
      g_extSinglesIndex = (i + 1) % EXT_SINGLES_COUNT;
      break;
    }
  }
}

void handleBurst(unsigned long nowMs) {
  if (!g_burstActive) {
    if (nowMs - lastBurstMs < BURST_PERIOD_MS) return;
    g_burstActive = true;
    g_burstIndex = 0;
    g_burstNextMs = nowMs;
    lastBurstMs = nowMs;
  }
  if (nowMs >= g_burstNextMs) {
    sendExt(*burstSeq[g_burstIndex]);
    g_burstIndex++;
    if (g_burstIndex >= BURST_SEQ_COUNT) {
      g_burstActive = false;
    } else {
      g_burstNextMs = nowMs + BURST_INTERFRAME_DELAY_MS;
    }
  }
}

// ----------------------
// WEB HANDLERS
// ----------------------

String profileName(ProfileMode m) {
  return (m == PROFILE_MH2P) ? String("mh2p") : String("mh2p+cluster");
}

void handleCmd() {
  if (!server.hasArg("c")) { server.send(400, "text/plain", "missing c"); return; }
  String cmd = server.arg("c");
  String out = steer_handleCommand(cmd);
  if (!out.length()) out = "OK\n";
  server.send(200, "text/plain", out);
}

void handleRoot() {
  String mode = (WiFi.getMode() & WIFI_MODE_AP) ? "AP" : "STA";
  String status = (WiFi.status() == WL_CONNECTED) ? "connected" : "not connected";
  String ip = (WiFi.status() == WL_CONNECTED) ? WiFi.localIP().toString() : WiFi.softAPIP().toString();
  String html;
  html += "<!doctype html><html><head><meta charset='utf-8'><title>MH2P CAN Simulator</title>";
  html += "<style>";
  html += "body{font-family:'Trebuchet MS','Gill Sans','Segoe UI',sans-serif;background:radial-gradient(circle at top right,#1a2433,transparent 50%),linear-gradient(135deg,#0f141b,#131c28 55%,#0f141b);color:#e8eef7;margin:0;padding:16px;}";
  html += "h1,h2,h3{margin:0 0 8px 0;letter-spacing:0.3px;}";
  html += ".card{background:#171f2a;border:1px solid #263244;border-radius:12px;padding:12px;margin:12px 0;box-shadow:0 10px 24px rgba(0,0,0,0.25);animation:riseIn .35s ease both;}";
  html += ".card:nth-of-type(2){animation-delay:.04s;}.card:nth-of-type(3){animation-delay:.08s;}.card:nth-of-type(4){animation-delay:.12s;}";
  html += ".row{display:flex;flex-wrap:wrap;gap:8px;align-items:center;}";
  html += ".row-between{justify-content:space-between;}";
  html += ".pill{display:inline-block;padding:4px 8px;border-radius:999px;font-size:12px;background:#243245;color:#c8d4e3;border:1px solid #2b3a4e;}";
  html += ".pill-on{background:#16392b;color:#8ff0c2;border-color:#1f8f5f;}";
  html += ".pill-off{background:#3a1a1a;color:#ffb3b3;border-color:#b04a4a;}";
  html += ".btn{display:inline-block;padding:8px 12px;border-radius:8px;text-decoration:none;color:#fff;background:#3b465a;border:1px solid #51637f;font-weight:600;}";
  html += ".btn-on{background:#1f8f5f;border-color:#2fb77a;}";
  html += ".btn-off{background:#823a3a;border-color:#b04a4a;}";
  html += "table{width:100%;border-collapse:collapse;font-size:12px;}";
  html += "th,td{border:1px solid #2a3a52;padding:6px;text-align:left;}";
  html += "th{background:#1b2636;}";
  html += "a{color:#a7c7ff;}";
  html += "@keyframes riseIn{from{opacity:0;transform:translateY(6px);}to{opacity:1;transform:translateY(0);}}";
  html += "</style></head><body>";
  html += "<h1>MH2P CAN Simulator</h1>";
  html += "<div class='card'>";
  html += "<div class='row row-between'><h2>Network</h2><span class='pill'>" + mode + "</span></div>";
  html += "<div class='row'>";
  html += "<span class='pill'>Status: " + status + "</span>";
  html += "<span class='pill'>SSID: " + String(WiFi.SSID()) + "</span>";
  html += "<span class='pill'>IP: " + ip + "</span>";
  html += "</div></div>";
  html += "<div class='card'>";
  html += "<h2>CAN Controls</h2>";
  html += "<p>Profile: <span class='pill'>" + profileName(currentProfile) + "</span></p>";
  html += "<form action='/set_profile' method='get'>";
  html += "<select name='mode'>";
  html += (currentProfile == PROFILE_MH2P) ? "<option value='mh2p' selected>mh2p</option>" : "<option value='mh2p'>mh2p</option>";
  html += (currentProfile == PROFILE_CLUSTER) ? "<option value='cluster' selected>mh2p+cluster</option>" : "<option value='cluster'>mh2p+cluster</option>";
  html += "</select> <button type='submit'>Apply</button></form>";
  html += "<div class='row'>";
  html += "<span class='pill'>TX: " + String(g_txEnabled ? "ON" : "OFF") + "</span>";
  html += "<span class='pill'>KL15: " + String(g_kl15On ? "ON" : "OFF") + "</span>";
  html += "<span class='pill'>Steering: " + String(g_steerEnabled ? "ON" : "OFF") + "</span>";
  html += "<span class='pill'>Bursts+Singles: " + String(g_mh2pExtrasEnabled ? "ON" : "OFF") + "</span>";
  html += "</div>";
  html += "<div class='row' style='margin-top:8px;'>";
  html += "<a class='btn " + String(g_txEnabled ? "btn-on" : "btn-off") + "' href='/toggle_tx'>Toggle TX</a>";
  html += "<a class='btn " + String(g_kl15On ? "btn-on" : "btn-off") + "' href='/toggle_kl15'>Toggle KL15</a>";
  html += "<a class='btn " + String(g_steerEnabled ? "btn-on" : "btn-off") + "' href='/toggle_steer'>Toggle Steering</a>";
  html += "<a class='btn " + String(g_mh2pExtrasEnabled ? "btn-on" : "btn-off") + "' href='/toggle_mh2p_extras'>Toggle Bursts + Singles</a>";
  html += "<a class='btn' href='/reset_counts'>Reset Counters</a>";
  html += "</div></div>";
  html += "<div class='card'>";
  html += "<div class='row row-between'><h2>Steering Injector</h2>";
  html += "<span class='pill " + String(g_steerEnabled ? "pill-on" : "pill-off") + "'>" + String(g_steerEnabled ? "ON" : "OFF") + "</span></div>";
  html += "<div class='row row-between' style='margin-top:6px;'>";
  html += "<span></span><a class='btn' href='/cmd?c=state'>State</a></div>";
  html += "<div class='row'>";
  html += "<a class='btn' href='/cmd?c=tap%20ok'>OK</a>";
  html += "<a class='btn' href='/cmd?c=tap%20left'>Left</a>";
  html += "<a class='btn' href='/cmd?c=tap%20right'>Right</a>";
  html += "<a class='btn' href='/cmd?c=tap%20view'>View</a>";
  html += "<a class='btn' href='/cmd?c=tap%20menu'>Menu</a>";
  html += "</div>";
  html += "<div class='row'>";
  html += "<a class='btn' href='/cmd?c=tap%20next'>Next</a>";
  html += "<a class='btn' href='/cmd?c=tap%20prev'>Prev</a>";
  html += "<a class='btn' href='/cmd?c=tap%20mute'>Mute</a>";
  html += "<a class='btn' href='/cmd?c=tap%20phone'>Phone</a>";
  html += "<a class='btn' href='/cmd?c=tap%20voice'>Voice</a>";
  html += "</div>";
  html += "<div class='row'>";
  html += "<a class='btn' href='/cmd?c=long%20ok'>Long OK</a>";
  html += "<a class='btn' href='/cmd?c=long%20left'>Long Left</a>";
  html += "<a class='btn' href='/cmd?c=long%20right'>Long Right</a>";
  html += "<a class='btn' href='/cmd?c=long%20view'>Long View</a>";
  html += "<a class='btn' href='/cmd?c=release'>Release</a>";
  html += "</div>";
  html += "<div class='row'>";
  html += "<a class='btn' href='/cmd?c=knob%20up'>Knob +</a>";
  html += "<a class='btn' href='/cmd?c=knob%20down'>Knob -</a>";
  html += "<a class='btn' href='/cmd?c=knob%20up2'>Knob ++</a>";
  html += "<a class='btn' href='/cmd?c=knob%20down2'>Knob --</a>";
  html += "</div>";
  html += "<form action='/cmd' method='get'>";
  html += "<input name='c' placeholder='cmd e.g. tap ok' size='30'/>";
  html += "<button type='submit'>Send</button></form>";
  html += "</div>";
  html += "<div class='card'><h3>CAN ID List</h3>";
  html += "<table border='1' cellpadding='4' cellspacing='0'>";
  html += "<tr><th>ID</th><th>Type</th><th>Enabled</th><th>Count</th><th>Action</th></tr>";
  for (int i = 0; i < ID_ENTRY_COUNT; i++) {
    html += "<tr>";
    html += "<td>" + String(idEntries[i].label) + "</td>";
    html += "<td>" + String(idEntries[i].ext ? "EXT" : "STD") + "</td>";
    html += "<td>" + String(idEntries[i].enabled ? "YES" : "NO") + "</td>";
    html += "<td>" + String(idEntries[i].count) + "</td>";
    html += "<td><a href='/toggle_id?i=" + String(i) + "'>Toggle</a></td>";
    html += "</tr>";
  }
  html += "</table></div>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

// ----------------------
// PROFILE CONTROL
// ----------------------

void applyProfile(ProfileMode m) {
  currentProfile = m;
  g_mh2pExtrasEnabled = (m == PROFILE_MH2P);
  unsigned long now = millis();
  phaseStartMs = now;
  seq3C0_started_mh2p = false;
  seq3C0_started_cluster = false;
  seq3C0_index_mh2p = 0;
  seq3C0_index_cluster = 0;
  last3C0Ms_mh2p = now;
  last3C0Ms_cluster = now;
  for (int i = 0; i < STD_FRAME_MH2P_COUNT; i++) {
    unsigned long offset = (stdFrames_mh2p[i].periodMs * i) / STD_FRAME_MH2P_COUNT;
    stdFrames_mh2p[i].lastSentMs = now - offset;
  }
  for (int i = 0; i < STD_FRAME_CLUSTER_COUNT; i++) {
    stdFrames_cluster[i] = clusterStdFramesConst[i];
    unsigned long offset = (stdFrames_cluster[i].periodMs * i) / STD_FRAME_CLUSTER_COUNT;
    stdFrames_cluster[i].lastSentMs = now - offset;
  }
  for (int i = 0; i < EXT_SINGLES_COUNT; i++) {
    unsigned long offset = (extSingles[i].periodMs * i) / EXT_SINGLES_COUNT;
    extSingles[i].lastSentMs = now - offset;
  }
  g_burstActive = false;
  g_burstIndex = 0;
  g_burstNextMs = 0;
  lastBurstMs = now;
}

// ----------------------
// SETUP
// ----------------------

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  const unsigned long wifiStart = millis();
  const unsigned long wifiTimeoutMs = 15000;
  while (WiFi.status() != WL_CONNECTED && (millis() - wifiStart) < wifiTimeoutMs) {
    delay(250);
  }
  if (WiFi.status() != WL_CONNECTED) {
    WiFi.disconnect(true);
    WiFi.mode(WIFI_MODE_AP);
    WiFi.softAP(FALLBACK_AP_SSID, FALLBACK_AP_PASS);
  }

  server.on("/", handleRoot);
  server.on("/toggle_tx", []() { g_txEnabled = !g_txEnabled; redirectHome(); });
  server.on("/toggle_kl15", []() { g_kl15On = !g_kl15On; redirectHome(); });
  server.on("/toggle_steer", []() { g_steerEnabled = !g_steerEnabled; redirectHome(); });
  server.on("/toggle_mh2p_extras", []() { g_mh2pExtrasEnabled = !g_mh2pExtrasEnabled; redirectHome(); });
  server.on("/toggle_id", []() {
    if (server.hasArg("i")) {
      int idx = server.arg("i").toInt();
      if (idx >= 0 && idx < ID_ENTRY_COUNT) idEntries[idx].enabled = !idEntries[idx].enabled;
    }
    redirectHome();
  });
  server.on("/reset_counts", []() {
    for (int i = 0; i < ID_ENTRY_COUNT; i++) idEntries[i].count = 0;
    redirectHome();
  });
  server.on("/cmd", handleCmd);
  server.on("/set_profile", []() {
    if (server.hasArg("mode")) {
      String m = server.arg("mode"); m.toLowerCase();
      if (m == "mh2p") applyProfile(PROFILE_MH2P);
      else applyProfile(PROFILE_CLUSTER);
    }
    redirectHome();
  });
  server.begin();

  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  g_config.tx_queue_len = 60;
  g_config.rx_queue_len = 20;
  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("TWAI driver install FAILED");
    while (true) { delay(1000); }
  }
  if (twai_start() != ESP_OK) {
    Serial.println("TWAI start FAILED");
    while (true) { delay(1000); }
  }
  driver_installed = true;
  Serial.println("TWAI init OK");

  applyProfile(currentProfile);
}

// ----------------------
// MAIN LOOP
// ----------------------

void loop() {
  server.handleClient();
  unsigned long nowMs = millis();
  unsigned long elapsed = nowMs - phaseStartMs;
  if (elapsed < INIT_DURATION_MS) {
    currentPhase = PHASE_INIT;
  } else if (elapsed < INIT_DURATION_MS + BOOTLOADER_DURATION_MS) {
    currentPhase = PHASE_BOOTLOADER;
  } else {
    currentPhase = PHASE_RUNTIME;
    if (elapsed > CYCLE_DURATION_MS) phaseStartMs = nowMs;
  }

  if (currentProfile == PROFILE_CLUSTER) tick3C0_cluster(nowMs);
  else tick3C0_mh2p(nowMs);

  if (currentProfile == PROFILE_CLUSTER) {
    for (int i = 0; i < STD_FRAME_CLUSTER_COUNT; i++) {
      if (nowMs - stdFrames_cluster[i].lastSentMs >= stdFrames_cluster[i].periodMs) {
        sendStd(stdFrames_cluster[i]);
        stdFrames_cluster[i].lastSentMs = nowMs;
      }
    }
  } else {
    for (int i = 0; i < STD_FRAME_MH2P_COUNT; i++) {
      if (nowMs - stdFrames_mh2p[i].lastSentMs >= stdFrames_mh2p[i].periodMs) {
        sendStd(stdFrames_mh2p[i]);
        stdFrames_mh2p[i].lastSentMs = nowMs;
      }
    }
  }

  if (g_mh2pExtrasEnabled) {
    sendExtPeriodic();
    handleBurst(nowMs);
  }
  steer_tick(nowMs);
}
