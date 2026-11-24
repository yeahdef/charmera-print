// Types shared by charmera_bridge.ino and printers.ino.
#pragma once
#include <Arduino.h>
#include <NimBLEDevice.h>

enum Family { F_NONE, F_CT320B, F_CT230B, F_ESCPOS, F_PHOMEMO, F_CAT, F_MXW01, F_P50, F_AE30, F_ISSC, F_AF30, F_BX1, F_NT861, F_HELD, F_GENERIC };

struct FamilyInfo {
  const char *name;
  int width;      // head width in dots
  int dpi;
  int maxRows;    // the firmware's trusted length
  int chunk;      // largest write
  int gapMs;      // wait after each unacknowledged write
};

// A printer seen while scanning.
struct Cand { NimBLEAddress dev; Family fam; int rssi; String name, addr; };
