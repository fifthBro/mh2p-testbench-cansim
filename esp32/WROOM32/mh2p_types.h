#pragma once
#include <Arduino.h>

// Types that must be visible to Arduino's auto-prototype generator.

struct FrameDef {
  uint32_t id;
  bool     ext;
  uint8_t  dlc;
  uint8_t  data[8];
  uint16_t periodMs;
};

enum Mh2pState : uint8_t {
  MH_INIT       = 0,
  MH_BOOTLOADER = 1,
  MH_RUNTIME    = 2
};
