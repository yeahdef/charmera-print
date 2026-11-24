// BLE printers: find any supported printer that is on and in range, identify
// its family, and print a 1-bit raster to it. No setup: the last printer used is
// preferred when it is advertising, otherwise the strongest known one wins.
//
// Protocols, pacing and identification rules are ported from magic-mate's
// printing module and matrix-dot-cam's PrinterGuess (Kotlin), where each was
// worked out on real hardware. Rules kept from there:
//   - identify by advertised name first, then advertised service, and only then
//     by a paper-free question after connecting;
//   - never send the cat question to an ESC/POS printer (it prints "Qx"), never
//     ESC/POS questions to a cat printer, and nothing at all down a Phomemo pipe
//     before printing;
//   - these printers take one connection and stop advertising while held.
//
// Families: Clabel CT-320B / CT-230B (TSPL), generic 58 mm ESC/POS (MPT-II and
// anything advertising 18F0), Phomemo M110 family, cat printers (AE30), MXW01,
// Marklife P50, BX1, NETUM NT-861, POS-8360 ("BlueTooth Printer"). Not yet:
// Munbyn FM226, Rongta RPP300.
//
// Raster input everywhere: 1 = black, leftmost dot in the top bit of each byte.

// Types are in printers.h (Arduino places its generated prototypes above any type defined in a .ino).
static const FamilyInfo FAM[] = {
  /* F_NONE    */ {"no printer", 576, 203, 900, 20, 30},
  /* F_CT320B  */ {"Clabel CT-320B", 576, 203, 900, 238, 0},   // back-pressure paced; ack first and last
  /* F_CT230B  */ {"Clabel CT-230B", 568, 300, 900, 200, 5},
  /* F_ESCPOS  */ {"ESC/POS 58mm", 384, 203, 1200, 244, 0},
  /* F_PHOMEMO */ {"Phomemo", 384, 203, 1200, 180, 5},
  /* F_CAT     */ {"cat printer", 384, 203, 1200, 128, 10},
  /* F_MXW01   */ {"MXW01", 384, 203, 1200, 180, 15},
  /* F_P50     */ {"Marklife P50", 384, 203, 1200, 180, 30},
  /* F_AE30    */ {"AE30 printer", 384, 203, 1200, 128, 30},    // cat or MXW01, decided after connecting
  /* F_ISSC    */ {"ISSC printer", 576, 203, 900, 200, 30},     // Clabel by its version answer
  /* F_AF30    */ {"AF30 printer", 384, 203, 1200, 128, 30},    // Phomemo or cat (an SC03h shows only AF30), by its services
  /* F_BX1     */ {"BX1", 384, 203, 1200, 202, 0},              // paced by byte rate, not gaps: see sendOnce
  /* F_NT861   */ {"NETUM NT-861", 576, 203, 900, 180, 0},      // stored-image conversation: see nt861Send
  /* F_HELD    */ {"80mm ESC/POS (POS-8360)", 576, 203, 900, 180, 0},   // held with drawer pulses: see heldSend
  /* F_GENERIC */ {"unknown printer", 384, 203, 1200, 180, 30},          // probed after connecting: see bindPipes
};
static int g_gapOverride = -1;   // an unknown printer resolved to a family still gets the safest pacing

static const char *ISSC_SVC = "49535343-fe7d-4ae5-8fa9-9fafd205e455";
static const char *ISSC_WR = "49535343-8841-43f4-a8d4-ecbe34729bb3";
static const char *ISSC_NT = "49535343-1e4d-4bd9-ba61-23c647249616";

static Family g_fam = F_NONE;
static String g_pname, g_paddr;
static NimBLEClient *g_pcli;
static NimBLERemoteCharacteristic *g_wr, *g_data, *g_nt;
static size_t g_pchunk = 20;

static bool printerReady() { return g_pcli && g_pcli->isConnected() && g_wr && g_fam != F_NONE; }
static int printerWidth() { return FAM[g_fam].width; }
static int printerMaxRows() { return FAM[g_fam].maxRows; }
static String printerName() { return g_fam == F_NONE ? String("no printer") : String(FAM[g_fam].name) + " " + g_pname; }

// ---------------------------------------------------------------- notifications
static uint8_t g_nbuf[2048];
static volatile size_t g_nlen;
static portMUX_TYPE g_nmux = portMUX_INITIALIZER_UNLOCKED;
static void onNotify(NimBLERemoteCharacteristic *, uint8_t *d, size_t n, bool) {
  portENTER_CRITICAL(&g_nmux);
  if (g_nlen + n > sizeof g_nbuf) g_nlen = 0;            // overflow: keep the newest
  memcpy(g_nbuf + g_nlen, d, n); g_nlen += n;
  portEXIT_CRITICAL(&g_nmux);
}
static void notifyClear() { portENTER_CRITICAL(&g_nmux); g_nlen = 0; portEXIT_CRITICAL(&g_nmux); }
// Wait until the notifications hold `pat` (`n` bytes); true if seen in time.
static bool notifyWait(const uint8_t *pat, size_t n, uint32_t ms) {
  uint32_t t0 = millis();
  for (;;) {
    portENTER_CRITICAL(&g_nmux);
    bool hit = false;
    for (size_t i = 0; i + n <= g_nlen && !hit; i++) hit = !memcmp(g_nbuf + i, pat, n);
    portEXIT_CRITICAL(&g_nmux);
    if (hit) return true;
    if (millis() - t0 >= ms) return false;
    delay(10);
  }
}
static void notifyCopy(uint8_t *out, size_t cap, size_t *got) {
  portENTER_CRITICAL(&g_nmux);
  *got = g_nlen < cap ? g_nlen : cap; memcpy(out, g_nbuf, *got);
  portEXIT_CRITICAL(&g_nmux);
}

// ---------------------------------------------------------------- helpers
static uint8_t crc8_07(const uint8_t *d, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; i++) {
    c ^= d[i];
    for (int b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}
static uint8_t rev8(uint8_t b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4; b = (b & 0xCC) >> 2 | (b & 0x33) << 2; b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

// One write, resent while the Bluetooth stack's send queue is full.
static bool bleWrite(NimBLERemoteCharacteristic *ch, const uint8_t *p, size_t n, bool ack, int *resends) {
  for (int tries = 0; tries < 400; tries++) {
    if (ch->writeValue(p, n, ack)) return true;
    if (!g_pcli || !g_pcli->isConnected()) return false;
    if (resends) (*resends)++;
    delay(4);
  }
  return false;
}

// Stream `len` bytes in the family's chunk size and pacing. The first write is
// acknowledged when the characteristic allows it: an idle link the printer has
// dropped still looks connected, and unacknowledged writes into it vanish.
static const char *streamTo(NimBLERemoteCharacteristic *ch, const uint8_t *p, size_t len, size_t chunk, int gapMs,
                            bool ackLast, int *resends, uint32_t bytesPerSec = 0) {
  bool noResp = ch->canWriteNoResponse();
  bool canAck = ch->canWrite();
  uint32_t t0 = millis();
  for (size_t i = 0; i < len; i += chunk) {
    if (bytesPerSec) {                                   // a steady rate: wait until this chunk is due
      uint32_t due = t0 + (uint32_t)((uint64_t)i * 1000 / bytesPerSec);
      while ((int32_t)(millis() - due) < 0) delay(1);
    }
    size_t n = len - i < chunk ? len - i : chunk;
    bool first = i == 0, last = i + n >= len;
    bool ack = !noResp || (canAck && (first || (ackLast && last)));
    if (!bleWrite(ch, p + i, n, ack, resends)) return first ? "stale" : "printer stopped accepting data";
    if (noResp && !ack && gapMs) delay(gapMs);
  }
  return nullptr;
}

// ---------------------------------------------------------------- scan and identify
static Cand g_cand[16];
static int g_ncand;
static String g_skip[8];
static int g_nskip;

static bool phomemoSerial(const String &n) {   // a paired Phomemo renames itself: q + 3 digits, >= 8 alphanumerics
  if (n.length() < 8 || tolower(n[0]) != 'q') return false;
  for (int i = 1; i <= 3; i++) if (!isdigit(n[i])) return false;
  for (size_t i = 0; i < n.length(); i++) if (!isalnum(n[i])) return false;
  return true;
}
// Printers magic-mate drives with conversations this port doesn't have yet,
// recognised by name so they are never mistaken for a generic printer.
static bool notYetSupported(const String &nm) {
  String u = nm; u.toUpperCase();
  return u.indexOf("RPP300") >= 0 || u.indexOf("FM226") >= 0;
}
static Family byName(const String &nm) {
  String u = nm; u.toUpperCase();
  if (u.indexOf("CT320B") >= 0 || u.indexOf("CT-320B") >= 0) return F_CT320B;
  if (u.indexOf("CT230B") >= 0 || u.indexOf("CT-230B") >= 0) return F_CT230B;
  if (u.indexOf("MXW01") >= 0) return F_MXW01;
  if (u.startsWith("P50")) return F_P50;
  if (u.indexOf("MPT-II") >= 0) return F_ESCPOS;
  if (u.indexOf("BX1") >= 0) return F_BX1;
  // Before the 18F0 rule: the NT-861 advertises 18F0 like a 58 mm ESC/POS
  // printer, but it is 80 mm and needs its stored-image protocol.
  if (u.indexOf("TM-M30") >= 0 || u.indexOf("NT-861") >= 0) return F_NT861;
  // The POS-8360's module name, shared with other printers: confirmed by its
  // ISSC service after connecting (see bindPipes).
  if (u.indexOf("BLUETOOTH PRINTER") >= 0 || u.indexOf("POS-8360") >= 0) return F_HELD;
  if (u.indexOf("PHOMEMO") >= 0 || u.indexOf("M110") >= 0 || u.indexOf("M120") >= 0 || u.indexOf("M220") >= 0 || phomemoSerial(nm)) return F_PHOMEMO;
  return F_NONE;
}
// A device not recognised as any family is only ever considered (and written
// to) when its name says printer.
static bool looksLikePrinter(const String &nm, const NimBLEAdvertisedDevice *) {
  String l = nm; l.toLowerCase();
  static const char *words[] = {"print", "pos", "receipt", "thermal", "label", "mpt", "zj-", "xp-", "goojprt", "peripage", "paperang", "memobird"};
  for (auto w : words) if (l.indexOf(w) >= 0) return true;
  // Not FF00 alone: it is a common vendor service on gadgets that aren't
  // printers, and an FF00 printer is sent Phomemo data without being asked.
  return false;
}

static Family byAdvert(const NimBLEAdvertisedDevice *d) {
  // In order of how specific each service is: cat printers (an SC03h) also
  // advertise AF30, the Phomemo vendor service, so AE30 has to win.
  static const struct { uint16_t u; Family f; } order[] = {{0xAE30, F_AE30}, {0x18F0, F_ESCPOS}, {0xAF30, F_AF30}};
  for (auto &o : order)
    if (d->isAdvertisingService(NimBLEUUID(o.u))) return o.f;
  if (d->isAdvertisingService(NimBLEUUID(ISSC_SVC))) return F_ISSC;
  return F_NONE;
}

class PrinterScanCB : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice *d) override {
    String nm;
    if (d->haveName()) for (char ch : String(d->getName().c_str())) if (ch >= 32 && ch < 127) nm += ch;   // names can carry junk bytes
    if (notYetSupported(nm)) {
      String addr = d->getAddress().toString().c_str();
      for (int i = 0; i < g_nskip; i++) if (g_skip[i] == addr) return;
      if (g_nskip < 8) { g_skip[g_nskip++] = addr; logf("  skipping \"%s\": not supported yet", nm.c_str()); }
      return;
    }
    Family f = byName(nm);
    if (f == F_NONE) f = byAdvert(d);
    if (f == F_NONE && looksLikePrinter(nm, d)) f = F_GENERIC;
    if (f == F_NONE) return;
    String addr = d->getAddress().toString().c_str();
    for (int i = 0; i < g_ncand; i++) if (g_cand[i].addr == addr) return;
    if (g_ncand >= 16) return;
    g_cand[g_ncand++] = {d->getAddress(), f, d->getRSSI(), nm, addr};
  }
};

static String g_prefName;   // set over serial to pick a printer by name (testing, several printers on)
static void printerPrefer(const String &frag) { g_prefName = frag; g_prefName.toUpperCase(); printerDrop(); }
static String savedPrinter() {
  if (!g_fat) return "";
  File f = FFat.open("/printer.txt", "r");
  if (!f) return "";
  String s = f.readString(); f.close(); s.trim();
  return s;
}
static void savePrinter(const String &addr) {
  if (!g_fat || addr == savedPrinter()) return;
  File f = FFat.open("/printer.txt", "w");
  if (f) { f.print(addr); f.close(); }
}

static void printerDrop() {
  if (g_pcli && g_pcli->isConnected()) g_pcli->disconnect();
  g_wr = g_data = g_nt = nullptr;
  g_fam = F_NONE;
}

// Paper-free question to an unnamed ISSC printer: the Clabel version query,
// `DD seq fn=2 sub=1 len=0 crc DD`, answered with text naming the model.
static Family askClabel() {
  uint8_t q[8] = {0xDD, 0x00, 0x02, 0x01, 0x00, 0x00, 0, 0xDD};
  q[6] = crc8_07(q + 1, 5);
  notifyClear();
  if (!bleWrite(g_wr, q, sizeof q, g_wr->canWrite(), nullptr)) return F_NONE;
  uint32_t t0 = millis();
  while (millis() - t0 < 1200) {
    uint8_t b[256]; size_t n;
    notifyCopy(b, sizeof b, &n);
    String s; for (size_t i = 0; i < n; i++) if (b[i] >= 32 && b[i] < 127) s += (char)b[i];
    if (s.indexOf("CT320B") >= 0) return F_CT320B;
    if (s.indexOf("CT230B") >= 0) return F_CT230B;
    delay(20);
  }
  return F_NONE;
}

// MXW01 status question `22 21 A1 00 01 00 00 crc FF`: any `22 21` answer means MXW01.
static bool askMxw01() {
  uint8_t q[9] = {0x22, 0x21, 0xA1, 0x00, 0x01, 0x00, 0x00, 0, 0xFF};
  q[7] = crc8_07(q + 6, 1);
  notifyClear();
  if (!bleWrite(g_wr, q, sizeof q, g_wr->canWrite(), nullptr)) return false;
  const uint8_t a[2] = {0x22, 0x21};
  return notifyWait(a, 2, 600);
}

static NimBLERemoteCharacteristic *charOf(NimBLERemoteService *s, uint16_t u) { return s ? s->getCharacteristic(NimBLEUUID(u)) : nullptr; }

// ESC/POS real-time status: an answer that is not just the question sent back.
static bool answersEscPos() {
  const uint8_t qs[2][3] = {{0x10, 0x04, 0x01}, {0x10, 0x04, 0x02}};
  for (auto &q : qs) {
    notifyClear();
    if (!bleWrite(g_wr, q, 3, g_wr->canWrite() && !g_wr->canWriteNoResponse(), nullptr)) return false;
    uint32_t t0 = millis();
    while (millis() - t0 < 600) {
      uint8_t b[64]; size_t n;
      notifyCopy(b, sizeof b, &n);
      size_t skip = (n >= 3 && !memcmp(b, q, 3)) ? 3 : 0;   // an echo of the question is not an answer
      if (n > skip) return true;
      delay(20);
    }
  }
  return false;
}

// An unknown printer: take the first standard printer pipe, then decide by
// what it answers. Nothing but a real-time status question is sent until it has
// answered as a printer.
static const char *bindGeneric(Family &f) {
  NimBLERemoteService *s;
  if ((s = g_pcli->getService(NimBLEUUID((uint16_t)0xFF00))) && charOf(s, 0xFF02)) {
    // A printer on the Phomemo pipe answers every question the same way, so
    // nothing is asked: it is treated as one (matrix-dot-cam's rule).
    g_wr = charOf(s, 0xFF02); g_nt = charOf(s, 0xFF03); if (!g_nt) g_nt = charOf(s, 0xFF01);
    f = F_PHOMEMO;
  } else {
    if ((s = g_pcli->getService(NimBLEUUID((uint16_t)0x18F0)))) { g_wr = charOf(s, 0x2AF1); g_nt = charOf(s, 0x2AF0); }
    else if ((s = g_pcli->getService(NimBLEUUID(ISSC_SVC)))) { g_wr = s->getCharacteristic(NimBLEUUID(ISSC_WR)); g_nt = s->getCharacteristic(NimBLEUUID(ISSC_NT)); }
    else if ((s = g_pcli->getService(NimBLEUUID((uint16_t)0xFFE0)))) { g_wr = charOf(s, 0xFFE1); g_nt = charOf(s, 0xFFE1); }
    if (!g_wr) return "unknown printer: no printer pipe found";
    if (g_nt && (g_nt->canNotify() || g_nt->canIndicate())) g_nt->subscribe(g_nt->canNotify(), onNotify);
    if (!answersEscPos()) return "unknown printer did not answer as ESC/POS; nothing sent";
    f = F_ESCPOS;
  }
  g_gapOverride = 30;
  logf("  unknown printer: printing as %s, 384 dots, safest pacing", FAM[f].name);
  return nullptr;
}

// Find the family's pipes on a connected printer; settles ambiguous families.
static const char *bindPipes(Family &f) {
  g_wr = g_data = g_nt = nullptr;
  g_gapOverride = -1;
  NimBLERemoteService *s = nullptr;
  if (f == F_GENERIC) {
    const char *e = bindGeneric(f);
    if (e) return e;
    if (f == F_PHOMEMO && g_nt && (g_nt->canNotify() || g_nt->canIndicate())) g_nt->subscribe(g_nt->canNotify(), onNotify);
    return nullptr;
  }
  if (f == F_AF30) {
    // AF30 is the Phomemo vendor advert, but an SC03h cat printer shows it too:
    // its real services decide.
    if (g_pcli->getService(NimBLEUUID((uint16_t)0xAE30))) f = F_AE30;
    else if (g_pcli->getService(NimBLEUUID((uint16_t)0xFF00))) f = F_PHOMEMO;
    else return "AF30 printer with neither AE30 nor FF00";
    logf("  AF30 printer resolved by its services to %s", f == F_AE30 ? "AE30 (cat/MXW01)" : "Phomemo");
  }
  switch (f) {
    case F_CT320B: case F_CT230B: case F_ISSC:
      s = g_pcli->getService(NimBLEUUID(ISSC_SVC));
      if (s) { g_wr = s->getCharacteristic(NimBLEUUID(ISSC_WR)); g_nt = s->getCharacteristic(NimBLEUUID(ISSC_NT)); }
      break;
    case F_ESCPOS:
      s = g_pcli->getService(NimBLEUUID((uint16_t)0x18F0));
      g_wr = charOf(s, 0x2AF1); g_nt = charOf(s, 0x2AF0);
      break;
    case F_PHOMEMO:
      s = g_pcli->getService(NimBLEUUID((uint16_t)0xFF00));
      g_wr = charOf(s, 0xFF02); g_nt = charOf(s, 0xFF03);
      break;
    case F_P50:
      s = g_pcli->getService(NimBLEUUID((uint16_t)0xFF00));
      g_wr = charOf(s, 0xFF02); g_nt = charOf(s, 0xFF01);
      break;
    case F_HELD:
      // It carries 18F0 too but only answers on ISSC. Without ISSC this is some
      // other printer behind the same module name: tried as a 58 mm one.
      s = g_pcli->getService(NimBLEUUID(ISSC_SVC));
      if (s) { g_wr = s->getCharacteristic(NimBLEUUID(ISSC_WR)); g_nt = s->getCharacteristic(NimBLEUUID(ISSC_NT)); break; }
      f = F_ESCPOS;
      s = g_pcli->getService(NimBLEUUID((uint16_t)0x18F0));
      g_wr = charOf(s, 0x2AF1); g_nt = charOf(s, 0x2AF0);
      break;
    case F_NT861:
      s = g_pcli->getService(NimBLEUUID((uint16_t)0x18F0));
      if (s) { g_wr = charOf(s, 0x2AF1); g_nt = charOf(s, 0x2AF0); }
      else { s = g_pcli->getService(NimBLEUUID((uint16_t)0xFF00)); g_wr = charOf(s, 0xFF02); g_nt = charOf(s, 0xFF03); if (!g_nt) g_nt = charOf(s, 0xFF01); }
      break;
    case F_BX1: {
      // The BX1 has two FF00 services; the first takes every write and does
      // nothing with it. Its real pipe is in the second.
      int seen = 0;
      for (NimBLERemoteService *svc : g_pcli->getServices(true))
        if (svc->getUUID() == NimBLEUUID((uint16_t)0xFF00) && seen++ == 1) s = svc;
      if (!s) return "BX1's second FF00 service not found";
      g_wr = charOf(s, 0xFF02); g_nt = charOf(s, 0xFF01);
      break;
    }
    case F_CAT: case F_MXW01: case F_AE30:
      s = g_pcli->getService(NimBLEUUID((uint16_t)0xAE30));
      g_wr = charOf(s, 0xAE01); g_nt = charOf(s, 0xAE02); g_data = charOf(s, 0xAE03);
      break;
    default: break;
  }
  if (!g_wr) return "printer's write pipe not found";
  // Subscribing comes before anything else: a Phomemo not being listened to
  // accepts and acknowledges jobs and silently never prints them.
  if (g_nt && (g_nt->canNotify() || g_nt->canIndicate())) g_nt->subscribe(g_nt->canNotify(), onNotify);
  if (f == F_ISSC) {
    f = askClabel();
    if (f == F_NONE) return "ISSC printer did not answer as a Clabel";
  }
  if (f == F_AE30) {
    f = (g_data && askMxw01()) ? F_MXW01 : F_CAT;   // a silent AE30 printer is a cat printer
  }
  if (f == F_MXW01 && !g_data) return "MXW01 data pipe (AE03) not found";
  return nullptr;
}

static const char *connectTo(Cand &c) {
  // A fresh client per printer: reusing one straight after dropping another
  // printer failed to connect and left the new printer half-connected (it stopped
  // advertising). The old client object is left alone rather than deleted.
  if (g_pcli && (g_pcli->isConnected() || g_paddr != c.addr)) { g_pcli->disconnect(); NimBLEDevice::deleteClient(g_pcli); g_pcli = nullptr; }
  if (!g_pcli) g_pcli = NimBLEDevice::createClient();
  uint32_t t0 = millis();
  if (!g_pcli->connect(c.dev)) { g_pcli->disconnect(); return "printer connect failed"; }
  Family f = c.fam;
  const char *err = bindPipes(f);
  if (err) { g_pcli->disconnect(); return err; }
  bool cp = g_pcli->updateConnParams(6, 12, 0, 400);     // fastest standard interval
  int mtu = g_pcli->getMTU();
  size_t room = mtu > 23 ? mtu - 3 : 20;
  g_fam = f;
  size_t want = g_gapOverride >= 0 ? 180 : (size_t)FAM[f].chunk;
  g_pchunk = want < room ? want : room;
  g_pname = c.name; g_paddr = c.addr;
  savePrinter(c.addr);
  logf("printer: %s \"%s\" %s rssi %d, connected in %lu ms, mtu %d, chunk %u, fast interval %d",
       FAM[f].name, c.name.c_str(), c.addr.c_str(), c.rssi, (unsigned long)(millis() - t0), mtu, (unsigned)g_pchunk, (int)cp);
  return nullptr;
}

// Scan only and list what was seen (testing).
static void printerList() {
  g_ncand = 0; g_nskip = 0;
  static PrinterScanCB cb;
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&cb, false);
  scan->setActiveScan(true); scan->setInterval(100); scan->setWindow(99);
  scan->getResults(3000, false);
  scan->clearResults();
  for (int i = 0; i < g_ncand; i++)
    Serial.printf("SEEN %s \"%s\" %s rssi %d\n", FAM[g_cand[i].fam].name, g_cand[i].name.c_str(), g_cand[i].addr.c_str(), g_cand[i].rssi);
  Serial.println("SCANEND");
}

// Make sure a printer is connected: keep the current link, else scan for every
// known printer and take the last one used, or the strongest.
static const char *printerEnsure() {
  if (g_pcli && g_pcli->isConnected() && g_wr && g_fam != F_NONE) return nullptr;
  g_wr = g_data = g_nt = nullptr; g_fam = F_NONE;
  g_ncand = 0; g_nskip = 0;
  uint32_t t0 = millis();
  static PrinterScanCB cb;
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&cb, false);
  scan->setActiveScan(true); scan->setInterval(100); scan->setWindow(99);
  scan->getResults(3000, false);
  scan->clearResults();
  logf("printer scan: %d candidate(s) in %lu ms", g_ncand, (unsigned long)(millis() - t0));
  if (!g_ncand) return "no printer found (is one on?)";
  String want = savedPrinter();
  // Last used first, then by signal strength.
  int order[16]; for (int i = 0; i < g_ncand; i++) order[i] = i;
  for (int i = 0; i < g_ncand; i++)
    for (int j = i + 1; j < g_ncand; j++) {
      Cand &a = g_cand[order[i]], &b = g_cand[order[j]];
      auto rank = [&](Cand &c) { String u = c.name + " " + FAM[c.fam].name; u.toUpperCase();
        return ((g_prefName.length() && u.indexOf(g_prefName) >= 0) ? 4 : 0) + (c.addr == want ? 2 : 0) + (c.fam != F_GENERIC ? 1 : 0); };
      int ra = rank(a), rb = rank(b);
      bool bFirst = rb > ra || (rb == ra && b.rssi > a.rssi);
      if (bFirst) { int t = order[i]; order[i] = order[j]; order[j] = t; }
    }
  const char *err = "no printer found";
  for (int k = 0; k < g_ncand; k++) {
    Cand &c = g_cand[order[k]];
    logf("trying %s \"%s\" %s rssi %d", FAM[c.fam].name, c.name.c_str(), c.addr.c_str(), c.rssi);
    err = connectTo(c);
    if (!err) return nullptr;
    logf("  %s", err);
  }
  return err;
}

// ---------------------------------------------------------------- job builders
// zlib with a 1 KB window (header 28 15), greedy LZ77 in one fixed-code block:
// the Marklife P50 refuses any other window ("ER", nothing printed).
struct BitOut {
  uint8_t *p; size_t n, cap; uint32_t acc; int cnt;
  void push(uint8_t v) { if (n < cap) p[n] = v; n++; }
  void put(uint32_t v, int bits) { acc |= v << cnt; cnt += bits; while (cnt >= 8) { push(acc & 0xFF); acc >>= 8; cnt -= 8; } }
  void code(uint32_t c, int bits) { uint32_t r = 0; for (int b = 0; b < bits; b++) r |= ((c >> b) & 1) << (bits - 1 - b); put(r, bits); }
  void lit(int v) { if (v < 144) code(0x30 + v, 8); else if (v < 256) code(0x190 + v - 144, 9); else if (v < 280) code(v - 256, 7); else code(0xC0 + v - 280, 8); }
  void align() { if (cnt > 0) push(acc & 0xFF); acc = 0; cnt = 0; }
  void byte(uint8_t v) { align(); push(v); }
};
static const uint16_t LBASE[29] = {3, 4, 5, 6, 7, 8, 9, 10, 11, 13, 15, 17, 19, 23, 27, 31, 35, 43, 51, 59, 67, 83, 99, 115, 131, 163, 195, 227, 258};
static const uint8_t LEXTRA[29] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 4, 4, 4, 4, 5, 5, 5, 5, 0};
static const uint16_t DBASE[30] = {1, 2, 3, 4, 5, 7, 9, 13, 17, 25, 33, 49, 65, 97, 129, 193, 257, 385, 513, 769, 1025, 1537, 2049, 3073, 4097, 6145, 8193, 12289, 16385, 24577};
static const uint8_t DEXTRA[30] = {0, 0, 0, 0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8, 8, 9, 9, 10, 10, 11, 11, 12, 12, 13, 13};
static uint32_t adler32(const uint8_t *d, size_t n) {
  uint32_t a = 1, b = 0;
  for (size_t i = 0; i < n; i++) { a = (a + d[i]) % 65521; b = (b + a) % 65521; }
  return (b << 16) | a;
}
static size_t zlib1k(const uint8_t *d, size_t n, uint8_t *out, size_t cap) {
  static int32_t *prev, *headAbs; static size_t prevCap;
  const int HASH = 1 << 13, WINDOW = 1024, MAXCHAIN = 64;
  if (n > prevCap) { free(prev); prev = (int32_t *)ps_malloc(n * sizeof(int32_t)); prevCap = prev ? n : 0; }
  if (!headAbs) headAbs = (int32_t *)ps_malloc(HASH * sizeof(int32_t));
  if (!prev || !headAbs) return 0;
  for (int i = 0; i < HASH; i++) headAbs[i] = -1;
  auto hash = [&](size_t i) { return (((uint32_t)d[i] << 10) ^ ((uint32_t)d[i + 1] << 5) ^ d[i + 2]) & (HASH - 1); };
  auto ins = [&](size_t i) { if (i + 2 >= n) return; uint32_t h = hash(i); prev[i] = headAbs[h]; headAbs[h] = (int32_t)i; };
  BitOut o{out, 0, cap, 0, 0};
  o.byte(0x28); o.byte(0x15);          // deflate, 1 KB window, no dictionary
  o.put(1, 1); o.put(1, 2);            // last block, fixed codes
  size_t i = 0;
  while (i < n) {
    int bestLen = 0, bestDist = 0;
    if (i + 2 < n) {
      int32_t c = headAbs[hash(i)]; int tries = MAXCHAIN;
      int limit = (int)((n - i) < 258 ? (n - i) : 258);
      while (c >= 0 && (int)(i - c) <= WINDOW && tries-- > 0) {
        int len = 0;
        while (len < limit && d[c + len] == d[i + len]) len++;
        if (len > bestLen) { bestLen = len; bestDist = (int)(i - c); if (len == limit) break; }
        c = prev[c];
      }
    }
    if (bestLen >= 3) {
      int k = 0; while (k < 28 && LBASE[k + 1] <= bestLen) k++;
      o.lit(257 + k); if (LEXTRA[k]) o.put(bestLen - LBASE[k], LEXTRA[k]);
      int j = 0; while (j < 29 && DBASE[j + 1] <= bestDist) j++;
      o.code(j, 5); if (DEXTRA[j]) o.put(bestDist - DBASE[j], DEXTRA[j]);
      for (int k2 = 0; k2 < bestLen; k2++) ins(i + k2);
      i += bestLen;
    } else { o.lit(d[i]); ins(i); i++; }
  }
  o.lit(256);
  o.align();
  uint32_t a = adler32(d, n);
  o.byte(a >> 24); o.byte(a >> 16); o.byte(a >> 8); o.byte(a);
  return o.n <= cap ? o.n : 0;
}

static Family g_builtFam = F_NONE;
static bool printerBuild(const uint8_t *r, int w, int h) {
  const FamilyInfo &F = FAM[g_fam];
  g_builtFam = g_fam;
  const int bpr = (w + 7) / 8;
  jobLen = 0;
  switch (g_fam == F_NONE ? F_CT320B : g_fam) {
    case F_CT320B: case F_CT230B: default: {
      // TSPL label mode: BITMAP mode 3 is raw deflate with set bit = black,
      // in bands of at most 40 rows (bigger bands get dropped).
      int dpi = F.dpi ? F.dpi : 203;
      int wt = (w * 254 + dpi / 2) / dpi, ht = (h * 254 + dpi / 2) / dpi;
      char head[200];
      int n = snprintf(head, sizeof head,
                       "SIZE %d.%d mm,%d.%d mm\r\nGAP 0 mm,0 mm\r\nDIRECTION 0,0\r\nREFERENCE 0,0\r\nDENSITY 8\r\nSPEED 2\r\nCLS\r\n",
                       wt / 10, wt % 10, ht / 10, ht % 10);
      jobAppend(head, n);
      static uint8_t packed[40 * 128 + 512];
      for (int y = 0; y < h; y += 40) {
        int rows = h - y < 40 ? h - y : 40;
        size_t in = (size_t)rows * bpr, out = sizeof packed;
        tdefl_init(deflator, nullptr, nullptr, 128);
        if (tdefl_compress(deflator, r + (size_t)y * bpr, &in, packed, &out, TDEFL_FINISH) != TDEFL_STATUS_DONE) return false;
        char bh[64];
        int bn = snprintf(bh, sizeof bh, "BITMAP 0,%d,%d,%d,3,%u,", y, bpr, rows, (unsigned)out);
        jobAppend(bh, bn); jobAppend(packed, out);
      }
      jobAppend("PRINT 1,1\r\n", 11);
      break;
    }
    case F_ESCPOS: {
      const uint8_t init[] = {0x1B, 0x40};
      const uint8_t gs[] = {0x1D, 0x76, 0x30, 0x00, (uint8_t)bpr, (uint8_t)(bpr >> 8), (uint8_t)h, (uint8_t)(h >> 8)};
      const uint8_t feed[] = {0x1B, 0x64, 0x04};
      jobAppend(init, sizeof init); jobAppend(gs, sizeof gs); jobAppend(r, (size_t)bpr * h); jobAppend(feed, sizeof feed);
      break;
    }
    case F_PHOMEMO: {
      // Envelope, the image as ONE GS v 0 block (a split leaves a white seam),
      // then the commit footer without which nothing prints.
      const uint8_t head[] = {0x1B, 0x4E, 0x0D, 0x03, 0x1B, 0x4E, 0x04, 0x08, 0x1F, 0x11, 0x0B};
      const uint8_t gs[] = {0x1D, 0x76, 0x30, 0x00, (uint8_t)bpr, (uint8_t)(bpr >> 8), (uint8_t)h, (uint8_t)(h >> 8)};
      const uint8_t foot[] = {0x1F, 0xF0, 0x05, 0x00, 0x1F, 0xF0, 0x03, 0x00};
      jobAppend(head, sizeof head); jobAppend(gs, sizeof gs); jobAppend(r, (size_t)bpr * h); jobAppend(foot, sizeof foot);
      break;
    }
    case F_CAT: {
      // `51 78 cmd 00 lenLE payload crc8 FF` frames; one A2 frame per row with
      // the LEFT dot in the LOW bit.
      auto frame = [&](uint8_t cmd, const uint8_t *pl, uint16_t n) {
        uint8_t hd[6] = {0x51, 0x78, cmd, 0x00, (uint8_t)n, (uint8_t)(n >> 8)};
        uint8_t tl[2] = {crc8_07(pl, n), 0xFF};
        jobAppend(hd, 6); jobAppend(pl, n); jobAppend(tl, 2);
      };
      const uint8_t q[] = {0x33}, en[] = {0xE0, 0x2E}, mode[] = {0x00}, spd[] = {0x19};
      const uint8_t start[] = {0xAA, 0x55, 0x17, 0x38, 0x44, 0x5F, 0x5F, 0x5F, 0x44, 0x38, 0x2C};
      const uint8_t feed[] = {0x64, 0x00};
      const uint8_t end[] = {0xAA, 0x55, 0x17, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x17};
      const uint8_t st[] = {0x00};
      frame(0xA4, q, 1); frame(0xAF, en, 2); frame(0xBE, mode, 1); frame(0xBD, spd, 1); frame(0xA6, start, sizeof start);
      uint8_t row[128];
      for (int y = 0; y < h; y++) {
        for (int i = 0; i < bpr; i++) row[i] = rev8(r[(size_t)y * bpr + i]);
        frame(0xA2, row, bpr);
      }
      frame(0xA1, feed, 2); frame(0xA6, end, sizeof end); frame(0xA3, st, 1);
      break;
    }
    case F_MXW01: {
      // Only the rows (LSB = left dot, at least 90); the commands go in printerSend.
      int rows = h < 90 ? 90 : h;
      for (int y = 0; y < rows; y++) {
        uint8_t row[128] = {0};
        if (y < h) for (int i = 0; i < bpr; i++) row[i] = rev8(r[(size_t)y * bpr + i]);
        jobAppend(row, bpr);
      }
      break;
    }
    case F_HELD:
    case F_NT861: {
      // Only the picture, scrubbed: the printer obeys real-time commands even
      // inside image data (a chance `10 04` makes it answer, others clear its
      // buffers or fire the drawer), so no byte that can begin a command is sent
      // as picture; each gains a dot instead.
      jobAppend(r, (size_t)bpr * h);
      for (size_t i = 0; i < jobLen; i++) {
        uint8_t b = job[i];
        if (b == 0x10 || b == 0x1B || b == 0x1C || b == 0x1D || b == 0x1F) job[i] = b | 0x20;
      }
      break;
    }
    case F_BX1: {
      // `64 cmd seq lenLE payload 00 00 00 00 9B`; seq counts on across jobs.
      static uint8_t seq;
      auto frame = [&](uint8_t cmd, const uint8_t *pl, uint16_t n) {
        seq++;
        uint8_t hd[5] = {0x64, cmd, seq, (uint8_t)n, (uint8_t)(n >> 8)};
        const uint8_t tl[5] = {0, 0, 0, 0, 0x9B};
        jobAppend(hd, 5); if (n) jobAppend(pl, n); jobAppend(tl, 5);
      };
      const uint8_t b10[] = {0x10}, b01[] = {0x01}, spd[] = {35}, den[] = {5}, cont[] = {0x01, 0x00};
      const uint8_t wid[] = {(uint8_t)bpr, (uint8_t)(bpr >> 8)}, feed[] = {144, 0};
      frame(0x12, nullptr, 0); frame(0x17, b10, 1); frame(0x17, b01, 1); frame(0x11, nullptr, 0);
      frame(0x0A, spd, 1); frame(0x09, den, 1); frame(0x28, cont, 2); frame(0x04, wid, 2);
      size_t total = (size_t)bpr * h, step = (size_t)bpr * 4;
      for (size_t at = 0; at < total; at += step) frame(0x00, r + at, (uint16_t)(total - at < step ? total - at : step));
      frame(0x02, feed, 2);
      break;
    }
    case F_P50: {
      static uint8_t *z; static size_t zCap;
      size_t raw = (size_t)bpr * h, need = raw + raw / 8 + 64;
      if (need > zCap) { free(z); z = (uint8_t *)ps_malloc(need); zCap = z ? need : 0; }
      if (!z) return false;
      size_t zn = zlib1k(r, raw, z, zCap);
      if (!zn) return false;
      const uint8_t start[] = {0x1F, 0xC0, 0x01, 0x00};
      const uint8_t head[] = {0x1F, 0x10, (uint8_t)(bpr >> 8), (uint8_t)bpr, (uint8_t)(h >> 8), (uint8_t)h,
                              (uint8_t)(zn >> 24), (uint8_t)(zn >> 16), (uint8_t)(zn >> 8), (uint8_t)zn};
      const uint8_t feed[] = {0x1B, 0x4A, 0x64}, end[] = {0x1F, 0xC0, 0x01, 0x01};
      jobAppend(start, 4); jobAppend(head, sizeof head); jobAppend(z, zn); jobAppend(feed, 3); jobAppend(end, 4);
      break;
    }
  }
  if (jobLen >= jobCap) { logf("job too big for the buffer"); return false; }
  logf("job %u bytes for %dx%d, %s", (unsigned)jobLen, w, h, F.name);
  return true;
}

// ---------------------------------------------------------------- send
// ---------------------------------------------------------------- NETUM NT-861
// Ported from magic-mate's StoredJob, where each step fixed a wrong print:
// store the picture (GS ( L fn 112), ask (GS I 1) and print it (fn 50) only once
// the printer answers, as a lost byte otherwise turns the rest into text; pace
// at 25 KB/s, half that if the printer echoes what it is sent.
static bool g_ntEchoes = false;
static const uint8_t NT_ASK[3] = {0x1D, 0x49, 0x01};
static const uint8_t NT_CLEAR[10] = {0x10, 0x14, 0x08, 0x01, 0x03, 0x14, 0x01, 0x06, 0x02, 0x08};
static bool ntRealTime(uint8_t a, uint8_t b) { return a == 0x10 && (b == 0x04 || b == 0x05 || b == 0x14); }

// Ask GS I 1 and wait for any answer. A printer that echoes sends the question
// back first, and its answer is the few bytes after the echo.
static bool askReply(const uint8_t *q, uint32_t ms, uint8_t *last = nullptr);
static bool ntAsk(uint32_t ms) { return askReply(NT_ASK, ms); }
static bool askReply(const uint8_t *q, uint32_t ms, uint8_t *last) {
  notifyClear();
  if (!bleWrite(g_wr, q, 3, false, nullptr)) return false;
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    uint8_t b[2048]; size_t n;
    notifyCopy(b, sizeof b, &n);
    int at = -1;
    for (int i = (int)n - 3; i >= 0; i--) if (!memcmp(b + i, q, 3)) { at = i; break; }
    if (at >= 0) { g_ntEchoes = true; size_t after = n - at - 3; if (after >= 1 && after <= 4) { if (last) *last = b[n - 1]; return true; } }
    else if (n >= 1 && n <= 4) { if (last) *last = b[n - 1]; return true; }
    delay(10);
  }
  return false;
}
// Bytes at no more than 25 KB/s (12.5 if the printer echoes).
static bool ntPour(const uint8_t *p, size_t len, int *resends) {
  uint32_t t0 = millis(), rate = g_ntEchoes ? 12500 : 25000;
  for (size_t i = 0; i < len; i += g_pchunk) {
    uint32_t due = t0 + (uint32_t)((uint64_t)i * 1000 / rate);
    while ((int32_t)(millis() - due) < 0) delay(1);
    size_t n = len - i < g_pchunk ? len - i : g_pchunk;
    if (!bleWrite(g_wr, p + i, n, false, resends)) return false;
  }
  return true;
}
static void ntFeed(uint8_t *out, size_t *n, int dots) {   // ESC J, two half-dots a dot
  while (dots > 0) { int now = dots < 127 ? dots : 127; out[(*n)++] = 0x1B; out[(*n)++] = 0x4A; out[(*n)++] = (uint8_t)(now * 2); dots -= now; }
}
static const char *nt861Send(int *resends) {
  const int bpr = 72, rows = (int)(jobLen / bpr);
  if (!ntAsk(2000)) {
    // It may be stuck inside a store waiting for bytes: fill it and clear.
    logf("NT-861 not answering: filling a store it may be stuck in, and clearing");
    static uint8_t zeros[180];
    memset(zeros, 0, sizeof zeros);
    for (size_t i = 0; i < 65535; i += sizeof zeros) if (!ntPour(zeros, sizeof zeros, resends)) return "stale";
    bleWrite(g_wr, NT_CLEAR, sizeof NT_CLEAR, false, nullptr);
    delay(500);
    if (!ntAsk(2000)) return "the NT-861 is not answering (turn it off and on?)";
  }
  // Reset, print area, then the store; a row more is declared if the header's
  // own numbers would read as a real-time command.
  int declared = rows;
  uint8_t head[30]; size_t hn;
  for (;;) {
    size_t length = (size_t)bpr * declared + 10; int dots = bpr * 8;
    const uint8_t h[] = {0x1B, 0x40, 0x1D, 0x4C, 0x00, 0x00, 0x1D, 0x57, 0x40, 0x02,
                         0x1D, 0x28, 0x4C, (uint8_t)length, (uint8_t)(length >> 8),
                         0x30, 0x70, 0x30, 0x01, 0x01, 0x31, (uint8_t)dots, (uint8_t)(dots >> 8), (uint8_t)declared, (uint8_t)(declared >> 8)};
    memcpy(head, h, sizeof h); hn = sizeof h;
    bool clash = false;
    for (size_t i = 11; i < hn; i++) if (ntRealTime(head[i - 1], head[i])) clash = true;
    if (!clash) break;
    declared++;
  }
  if (ntRealTime(head[hn - 1], job[0])) job[0] = 0;
  size_t pad = (size_t)(declared - rows) * bpr;
  for (int attempt = 1; attempt <= 3; attempt++) {
    if (!ntPour(head, hn, resends) || !ntPour(job, jobLen, resends)) return "printer stopped accepting data";
    static uint8_t blank[72];
    memset(blank, 0, sizeof blank);
    for (size_t i = 0; i < pad; i += bpr) if (!ntPour(blank, bpr, resends)) return "printer stopped accepting data";
    if (ntAsk(2200)) {
      // The print command and a feed in one small write, then wait for the
      // paper to stop; then feed to the blade and cut.
      uint8_t go[32]; size_t n = 0;
      const uint8_t print[] = {0x1D, 0x28, 0x4C, 0x02, 0x00, 0x30, 0x32};
      memcpy(go, print, sizeof print); n = sizeof print;
      ntFeed(go, &n, 12 + 48);
      if (!bleWrite(g_wr, go, n, false, resends)) return "printer stopped accepting data";
      if (!ntAsk(6000 + (uint32_t)rows * 6)) return "the NT-861 didn't finish the print";
      uint8_t cut[16]; n = 0;
      ntFeed(cut, &n, 84 - 48);
      cut[n++] = 0x1D; cut[n++] = 0x56; cut[n++] = 0x01;
      if (!bleWrite(g_wr, cut, n, false, resends)) return "printer stopped accepting data";
      ntAsk(6000);
      if (attempt > 1) logf("NT-861 took the picture on try %d", attempt);
      return nullptr;
    }
    // A store that lost bytes is still waiting for them: feed it zeros, then clear.
    logf("NT-861: no answer after the picture (try %d), filling and clearing", attempt);
    static uint8_t zeros[180];
    memset(zeros, 0, sizeof zeros);
    size_t total = hn + jobLen + pad;
    for (size_t i = 0; i < total; i += sizeof zeros) if (!ntPour(zeros, sizeof zeros, resends)) return "printer stopped accepting data";
    bleWrite(g_wr, NT_CLEAR, sizeof NT_CLEAR, false, nullptr);
    delay(500);
  }
  return "the NT-861 kept losing the picture";
}

// ---------------------------------------------------------------- POS-8360 ("BlueTooth Printer")
// Ported from magic-mate's HeldJob: an 80 mm printer that prints a GS v 0
// raster as it arrives, faster than Bluetooth delivers it, leaving light bands.
// Cash drawer pulses (ESC p) ahead of the picture keep it busy, half a second
// each, while it goes on taking bytes, so the whole card has arrived before it
// prints. It answers GS r 1 once everything (the cut included) is done.
static const uint8_t HELD_PAPER[3] = {0x1D, 0x72, 0x01};
static uint32_t g_heldRate = 14000;            // measured link rate, B/s
static bool heldUnstick(int *resends) {
  // A raster short of bytes is still waiting for them: feed it nothing.
  static uint8_t zeros[180];
  memset(zeros, 0, sizeof zeros);
  for (size_t fed = 0; fed < 52000; fed += 8000) {
    for (size_t i = 0; i < 8000; i += sizeof zeros) if (!ntPour(zeros, sizeof zeros, resends)) return false;
    if (askReply(HELD_PAPER, 2000)) return true;
  }
  return false;
}
static const char *heldSend(int *resends) {
  const int bpr = 72, rows = (int)(jobLen / bpr);
  uint8_t st = 0;
  if (!askReply(HELD_PAPER, 2000, &st)) {
    logf("POS-8360 not answering: feeding it blank");
    if (!heldUnstick(resends) || !askReply(HELD_PAPER, 2000, &st)) return "the printer is not answering (turn it off and on?)";
  }
  if ((st & 0x0C) == 0x0C) return "the printer is out of paper";
  const uint8_t init[] = {0x1B, 0x40}, cut[] = {0x1D, 0x56, 0x01};
  const uint8_t gs[] = {0x1D, 0x76, 0x30, 0x00, (uint8_t)bpr, 0, (uint8_t)rows, (uint8_t)(rows >> 8)};
  const uint8_t feedHead[] = {0x1D, 0x76, 0x30, 0x00, 1, 0, 92, 0};   // 92 blank rows: blade 80 + edge 12
  static uint8_t feedRows[92];
  memset(feedRows, 0, sizeof feedRows);
  for (int attempt = 1; attempt <= 2; attempt++) {
    size_t picture = sizeof gs + jobLen;
    int holds = 0;
    if (picture >= 2000) { uint32_t ms = (uint32_t)((uint64_t)picture * 1100 / g_heldRate); holds = (int)((ms + 499) / 500); if (holds < 1) holds = 1; if (holds > 20) holds = 20; }
    const uint8_t hold[] = {0x1B, 0x70, 0x00, 0x01, 0xFF};
    uint32_t t0 = millis();
    bool ok = ntPour(init, sizeof init, resends);
    for (int i = 0; i < holds && ok; i++) ok = ntPour(hold, sizeof hold, resends);
    ok = ok && ntPour(gs, sizeof gs, resends) && ntPour(job, jobLen, resends) &&
         ntPour(feedHead, sizeof feedHead, resends) && ntPour(feedRows, sizeof feedRows, resends) && ntPour(cut, sizeof cut, resends);
    if (!ok) return attempt == 1 ? "stale" : "printer stopped accepting data";
    uint32_t took = millis() - t0 + 1;
    size_t sent = picture + holds * 5 + 2 + 8 + 92 + 3;
    if (sent >= 8000) { uint32_t r = (uint32_t)((uint64_t)sent * 1000 / took); g_heldRate = r < 4000 ? 4000 : r > 25000 ? 25000 : r; }
    uint32_t heldMs = holds * 500, wait = (heldMs > took ? heldMs - took : 0) + rows * 3 + 6000;
    if (askReply(HELD_PAPER, wait)) { if (attempt > 1) logf("POS-8360 printed on try %d", attempt); return nullptr; }
    // Lost bytes: the raster is still waiting. Feed it blank, cut that piece
    // off, and print the card again.
    logf("POS-8360: no answer after the picture (try %d)", attempt);
    if (!heldUnstick(resends)) return "the printer stopped answering";
    ntPour(feedHead, sizeof feedHead, resends); ntPour(feedRows, sizeof feedRows, resends); ntPour(cut, sizeof cut, resends);
    askReply(HELD_PAPER, 6000);
  }
  return "the printer kept losing the picture";
}

static const char *sendOnce(int *resends) {
  const FamilyInfo &F = FAM[g_fam];
  if (g_fam == F_NT861) return nt861Send(resends);
  if (g_fam == F_HELD) return heldSend(resends);
  if (g_fam == F_MXW01) {
    // A conversation: intensity, print request (wait for its answer), rows on
    // AE03, flush, then wait for "printed".
    auto cmd = [&](uint8_t id, const uint8_t *pl, uint16_t n) -> bool {
      uint8_t f[32];
      f[0] = 0x22; f[1] = 0x21; f[2] = id; f[3] = 0; f[4] = (uint8_t)n; f[5] = (uint8_t)(n >> 8);
      memcpy(f + 6, pl, n); f[6 + n] = crc8_07(pl, n); f[7 + n] = 0xFF;
      return bleWrite(g_wr, f, 8 + n, g_wr->canWrite() && !g_wr->canWriteNoResponse(), resends);
    };
    int rows = (int)(jobLen / 48);
    const uint8_t dark[] = {0x5D};
    if (!cmd(0xA2, dark, 1)) return "stale";
    delay(20);
    const uint8_t req[] = {(uint8_t)rows, (uint8_t)(rows >> 8), 0x30, 0x00};
    notifyClear();
    if (!cmd(0xA9, req, 4)) return "printer stopped accepting data";
    const uint8_t ok[] = {0x22, 0x21, 0xA9};
    if (!notifyWait(ok, 3, 3000)) return "MXW01 did not say it was ready (paper? too hot?)";
    const char *e = streamTo(g_data, job, jobLen, g_pchunk, F.gapMs, false, resends);
    if (e) return e;
    const uint8_t fl[] = {0x00};
    if (!cmd(0xAD, fl, 1)) return "printer stopped accepting data";
    const uint8_t done[] = {0x22, 0x21, 0xAA};
    notifyWait(done, 3, (uint32_t)rows * 45 + 3000);   // paper is slow; don't start the next card early
    return nullptr;
  }
  if (g_fam == F_P50) {
    // The P50 answers each job on FF01: "AA\r\n" when it has taken it, "ER" when
    // it refused it (a bad stream, for one) and printed nothing.
    notifyClear();
    const char *e = streamTo(g_wr, job, jobLen, g_pchunk, F.gapMs, false, resends);
    if (e) return e;
    const uint8_t aa[] = {0xAA, 0x0D, 0x0A}, er[] = {'E', 'R'};
    uint32_t t0 = millis();
    while (millis() - t0 < 8000) {
      if (notifyWait(aa, 3, 0)) { logf("P50 took the job (%lu ms after the last byte)", (unsigned long)(millis() - t0)); return nullptr; }
      if (notifyWait(er, 2, 0)) return "the P50 refused the job (ER)";
      delay(20);
    }
    logf("P50 sent no answer within 8 s");
    return nullptr;
  }
  if (g_fam == F_BX1)   // a steady 5,600 B/s: 30 ms gaps lose rows, slower leaves bands
    return streamTo(g_wr, job, jobLen, g_pchunk, 0, false, resends, 5600);
  return streamTo(g_wr, job, jobLen, g_pchunk, g_gapOverride >= 0 ? g_gapOverride : F.gapMs, g_fam == F_CT320B, resends);
}

static const char *printerSend() {
  if (!jobLen) return "no job";
  const char *err = printerEnsure();
  if (err) return err;
  if (g_fam != g_builtFam) return "the job was built for a different printer; print again";
  uint32_t t0 = millis();
  int resends = 0;
  err = sendOnce(&resends);
  if (err && !strcmp(err, "stale")) {
    // The link had died quietly. Reconnect (same printer if it is still there)
    // and send again; the job was built for this family's width, so a different
    // printer would need a rebuild.
    logf("printer link was stale, reconnecting");
    Family was = g_fam;
    printerDrop();
    err = printerEnsure();
    if (err) return err;
    if (g_fam != was) return "a different printer answered; send again";
    err = sendOnce(&resends);
    if (err && !strcmp(err, "stale")) err = "printer did not acknowledge";
  }
  if (err) { printerDrop(); return err; }
  logf("sent %u bytes in %lu ms to %s (%u-byte chunks, %d resends)", (unsigned)jobLen, (unsigned long)(millis() - t0),
       FAM[g_fam].name, (unsigned)g_pchunk, resends);
  return nullptr;
}
