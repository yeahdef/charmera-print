// Charmera -> M5Stack PaperColor -> Clabel CT-320B bridge.
//
// The PaperColor is the USB host for the camera (webcam mode, full speed, no
// VBUS), grabs one MJPEG frame, dithers it to 576 dots wide and sends a TSPL
// job to the printer over BLE.
//
// Boards: M5Stack PaperColor (e-ink, three keys) and M5StickS3 (colour LCD,
// front + side key); the board is detected at run time.
// Build: m5stack:esp32:m5stack_papercolor or m5stack:esp32:m5stack_sticks3,
//        USB Mode = Hardware CDC and JTAG, CDC On Boot = Enabled, PSRAM = OPI,
//        a partition scheme with a FAT partition (for the saved log).
//
// Two modes, chosen at boot:
//   serial mode  when a computer is on the USB port (or Button A asked for it):
//                normal USB serial; send 'd' to dump the log of the last run,
//                'j' + length + JPEG to build a job from a file, 'p' to print it.
//                Unplugging the computer restarts into host mode.
//   host mode    otherwise: USB host for the camera.
//                PaperColor: Up (BtnA) serial mode, Down (BtnB) webcam
//                photo + print, third key (BtnC) print again.
//                StickS3: front (BtnA) print again, side (BtnB) serial mode.
//                Holding the print-again key cycles the dither mode.
//                A UVC "button pressed" status event from the camera also
//                triggers a photo.
#include <M5Unified.h>
#include <stdarg.h>
#include "usb/usb_host.h"
#include <NimBLEDevice.h>
#include "printers.h"
#include "miniz.h"
#include <FFat.h>
#include <JPEGDEC.h>
#include "ff.h"
#include "diskio_impl.h"
#include "soc/rtc_cntl_reg.h"

// The ROM deflate compressor and the JPEG decoder both need far more stack than
// the default 8 KB Arduino loop task has.
SET_LOOP_TASK_STACK_SIZE(48 * 1024);

// ---------------------------------------------------------------- log
// Kept in RAM that survives a software restart, so serial mode can dump what
// happened in host mode.
#define LOG_CAP 28000
#define LOG_MAGIC 0xC0FFEE42u
static __NOINIT_ATTR char g_log[LOG_CAP];
static __NOINIT_ATTR uint32_t g_logMagic;
static __NOINIT_ATTR uint32_t g_logLen;
static __NOINIT_ATTR uint32_t g_forceSerial;
static bool g_serialMode = false;

static void logf(const char *fmt, ...) {
  char line[400];
  int n = snprintf(line, sizeof line, "%7lu ", (unsigned long)millis());
  va_list ap; va_start(ap, fmt);
  n += vsnprintf(line + n, sizeof line - n - 2, fmt, ap);
  va_end(ap);
  if (n > (int)sizeof line - 2) n = sizeof line - 2;
  line[n++] = '\n';
  if (g_logLen + n < LOG_CAP) { memcpy(g_log + g_logLen, line, n); g_logLen += n; }
  if (g_serialMode) Serial.write((const uint8_t *)line, n);
}

// The RAM log is lost on a power cycle, so a copy goes to the FAT partition
// after every shot and camera event.
static bool g_fat = false;
static void saveLog() {
  if (!g_fat) return;
  File f = FFat.open("/log.txt", "w");
  if (f) { f.write((const uint8_t *)g_log, g_logLen); f.close(); }
}
static void loadLog() {
  if (!g_fat) return;
  File f = FFat.open("/log.txt", "r");
  if (f) { g_logLen = f.read((uint8_t *)g_log, LOG_CAP - 1); f.close(); }
}

static void logHex(const char *tag, const uint8_t *p, int n) {
  char line[330];
  for (int off = 0; off < n; off += 96) {
    int m = n - off < 96 ? n - off : 96, k = 0;
    for (int i = 0; i < m; i++) k += snprintf(line + k, sizeof line - k, "%02x", p[off + i]);
    logf("%s[%d] %s", tag, off, line);
  }
}

// ---------------------------------------------------------------- screen
static String g_status;
static bool g_epaper = false;   // PaperColor: every full redraw takes seconds
static bool g_stick = false;    // StickS3: two keys, small fast LCD
static void screen(const String &text) {
  M5.Display.startWrite();
  M5.Display.fillScreen(TFT_WHITE);
  M5.Display.setTextColor(TFT_BLACK);
  M5.Display.setTextSize(g_stick ? 1.5f : 2);
  M5.Display.setCursor(0, g_stick ? 4 : 8);
  M5.Display.print(text);
  M5.Display.endWrite();
  M5.Display.display();
}
static void beep(int freq, int ms) { M5.Speaker.tone(freq, ms); }

// ---------------------------------------------------------------- USB host
static usb_host_client_handle_t client;
static usb_device_handle_t dev;
static volatile uint8_t newAddr = 0;
static volatile bool devGone = false;
static volatile uint32_t goneAt = 0;
static bool camReady = false;
static bool diskReady = false;

static usb_transfer_t *ctrlX;
static volatile bool ctrlDone;

// what the config descriptor says
static uint16_t bcdUVC = 0x0100;
static int vcIf = -1, vsIf = -1;
static uint8_t intrEp = 0, isoEp = 0;
static struct { uint8_t alt; uint16_t mps; } alts[8];
static int nAlts = 0;
static int mscIf = -1;                     // mass storage (SD card as a USB disk)
static uint8_t mscIn = 0, mscOut = 0;
static uint16_t mscMps = 64;
static uint8_t fmtIndex = 0, frameIndex = 0;
static uint16_t frameW = 0, frameH = 0;
static uint32_t frameInterval = 0;

// status (interrupt) endpoint
static usb_transfer_t *intrX;
static int intrLogged = 0;
static volatile bool trigger = false;

// streaming
#define ISO_XFERS 4
#define ISO_PKTS 8
#define FRAME_CAP (2 * 1024 * 1024)
#define WANT_FRAMES 4
static usb_transfer_t *isoX[ISO_XFERS];
static volatile bool streaming = false;
static volatile int isoPending = 0;
static uint16_t isoMps = 0;
static uint8_t *frameBuf, *keepBuf;
static size_t frameLen = 0, keepLen = 0;
static int lastFid = -1, goodFrames = 0, badFrames = 0, pktErr = 0, pktOk = 0;
static bool frameErr = false;
static volatile bool captured = false;
static char g_why[120];
// stream diagnostics
static int dHdrOnly, dPayloadPkts, dEof, dFidToggles, dBadHdr, dDumped, dDumpedData;
static uint32_t dPayloadBytes;
#define WHY(...) do { snprintf(g_why, sizeof g_why, __VA_ARGS__); logf("%s", g_why); } while (0)

static void clientCb(const usb_host_client_event_msg_t *msg, void *) {
  if (msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) newAddr = msg->new_dev.address;
  else if (msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) { devGone = true; goneAt = millis(); }
}
static void libTask(void *) {
  for (;;) { uint32_t flags; usb_host_lib_handle_events(portMAX_DELAY, &flags); }
}
static void ctrlCb(usb_transfer_t *) { ctrlDone = true; }

// Control transfer; returns bytes transferred in the data stage, or <0.
static int ctrl(uint8_t bmReq, uint8_t req, uint16_t val, uint16_t idx, uint8_t *data, uint16_t len) {
  usb_setup_packet_t *s = (usb_setup_packet_t *)ctrlX->data_buffer;
  s->bmRequestType = bmReq; s->bRequest = req; s->wValue = val; s->wIndex = idx; s->wLength = len;
  if (!(bmReq & 0x80) && len) memcpy(ctrlX->data_buffer + 8, data, len);
  ctrlX->num_bytes = 8 + len;
  ctrlX->device_handle = dev;
  ctrlX->bEndpointAddress = 0;
  ctrlX->callback = ctrlCb;
  ctrlDone = false;
  esp_err_t e = usb_host_transfer_submit_control(client, ctrlX);
  if (e != ESP_OK) { logf("ctrl submit err 0x%x", e); return -1; }
  uint32_t t0 = millis();
  while (!ctrlDone && millis() - t0 < 2000) usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
  if (!ctrlDone) { logf("ctrl timeout req %02x val %04x", req, val); return -2; }
  if (ctrlX->status != USB_TRANSFER_STATUS_COMPLETED) {
    logf("ctrl req %02x val %04x idx %04x status %d", req, val, idx, ctrlX->status);
    return -3;
  }
  int n = ctrlX->actual_num_bytes - 8;
  if ((bmReq & 0x80) && n > 0) memcpy(data, ctrlX->data_buffer + 8, n < len ? n : len);
  return n;
}

static uint32_t le32(const uint8_t *p) { return p[0] | p[1] << 8 | p[2] << 16 | (uint32_t)p[3] << 24; }
static void put32(uint8_t *p, uint32_t v) { p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24; }

static void parseConfig(const uint8_t *p, int total) {
  int curIf = -1, curAlt = 0, curSub = 0;
  bool inMjpeg = false;
  int bestScore = 1 << 30;
  nAlts = 0; vcIf = vsIf = -1; intrEp = isoEp = 0; fmtIndex = frameIndex = 0;
  mscIf = -1; mscIn = mscOut = 0;
  bool inMsc = false;
  for (int off = 0; off + 2 <= total && p[off] >= 2; off += p[off]) {
    const uint8_t *d = p + off;
    uint8_t len = d[0], type = d[1];
    if (type == 0x04 && len >= 9) {
      curIf = d[2]; curAlt = d[3]; curSub = (d[5] == 0x0E) ? d[6] : 0;
      inMsc = d[5] == 0x08 && d[7] == 0x50;            // mass storage, bulk-only transport
      if (inMsc && mscIf < 0) mscIf = curIf;
      if (curSub == 1 && vcIf < 0) vcIf = curIf;
      if (curSub == 2 && vsIf < 0) vsIf = curIf;
    } else if (type == 0x05 && len >= 7) {
      uint8_t ep = d[2], attr = d[3]; uint16_t mps = (d[4] | d[5] << 8) & 0x7FF;
      if (inMsc && curIf == mscIf && (attr & 3) == 2) { if (ep & 0x80) { mscIn = ep; mscMps = mps; } else mscOut = ep; }
      if (curSub == 1 && (attr & 3) == 3 && (ep & 0x80)) intrEp = ep;
      if (curSub == 2 && curIf == vsIf && (attr & 3) == 1 && (ep & 0x80)) {
        isoEp = ep;
        if (nAlts < 8) { alts[nAlts].alt = curAlt; alts[nAlts].mps = mps; nAlts++; }
      }
    } else if (type == 0x24 && len >= 3) {
      uint8_t sub = d[2];
      if (curSub == 1 && sub == 0x01 && len >= 5) bcdUVC = d[3] | d[4] << 8;
      if (curSub == 2) {
        if (sub == 0x06 && len >= 4) { inMjpeg = true; if (!fmtIndex) fmtIndex = d[3]; logf("format %u MJPEG, %u frames", d[3], d[4]); }
        else if (sub == 0x04 && len >= 4) { inMjpeg = false; logf("format %u uncompressed, %u frames", d[3], d[4]); }
        else if ((sub == 0x07 || sub == 0x05) && len >= 26) {
          uint16_t w = d[5] | d[6] << 8, h = d[7] | d[8] << 8;
          uint32_t defInt = le32(d + 21), maxInt = defInt;
          uint8_t nInt = d[25];
          if (nInt == 0 && len >= 38) maxInt = le32(d + 30);
          for (int i = 0; i < nInt && 26 + 4 * i + 4 <= len; i++) { uint32_t v = le32(d + 26 + 4 * i); if (v > maxInt) maxInt = v; }
          logf(" frame %u %ux%u default interval %lu, slowest %lu (%s)", d[3], w, h,
               (unsigned long)defInt, (unsigned long)maxInt, sub == 0x07 ? "MJPEG" : "raw");
          if (sub == 0x07 && inMjpeg) {
            int score = abs((int)w - 640);
            if (score < bestScore) { bestScore = score; frameIndex = d[3]; frameW = w; frameH = h; frameInterval = maxInt; }
          }
        }
      }
    }
  }
}

static void intrCb(usb_transfer_t *t) {
  if (t->status == USB_TRANSFER_STATUS_COMPLETED && t->actual_num_bytes > 0) {
    if (intrLogged < 40) { intrLogged++; logHex("status", t->data_buffer, t->actual_num_bytes); }
    const uint8_t *p = t->data_buffer;
    // UVC status packet from a VideoStreaming interface: button pressed
    if (t->actual_num_bytes >= 4 && (p[0] & 0x0F) == 2 && p[2] == 0 && p[3] == 1) trigger = true;
  }
  if (camReady && t->status != USB_TRANSFER_STATUS_NO_DEVICE && t->status != USB_TRANSFER_STATUS_CANCELED)
    usb_host_transfer_submit(t);
}

static void openCamera(uint8_t addr) {
  esp_err_t oe = ESP_FAIL;
  for (int i = 0; i < 20; i++) {                         // the camera can need a moment after enumeration
    oe = usb_host_device_open(client, addr, &dev);
    if (oe == ESP_OK) { if (i) logf("device open succeeded on try %d", i + 1); break; }
    usb_host_client_handle_events(client, pdMS_TO_TICKS(25));
  }
  if (oe != ESP_OK) { logf("device open failed: %s (0x%x), address %u", esp_err_to_name(oe), oe, addr); dev = nullptr; return; }
  const usb_device_desc_t *dd;
  usb_host_get_device_descriptor(dev, &dd);
  usb_device_info_t info;
  usb_host_device_info(dev, &info);
  logf("device %04x:%04x speed %d", dd->idVendor, dd->idProduct, info.speed);
  const usb_config_desc_t *cfg;
  if (usb_host_get_active_config_descriptor(dev, &cfg) != ESP_OK) { logf("no config descriptor"); return; }
  logHex("cfg", (const uint8_t *)cfg, cfg->wTotalLength);
  parseConfig((const uint8_t *)cfg, cfg->wTotalLength);
  logf("uvc %04x vcIf %d vsIf %d intrEp %02x isoEp %02x fmt %u frame %u %ux%u interval %lu alts %d",
       bcdUVC, vcIf, vsIf, intrEp, isoEp, fmtIndex, frameIndex, frameW, frameH, (unsigned long)frameInterval, nAlts);
  for (int i = 0; i < nAlts; i++) logf(" alt %u mps %u", alts[i].alt, alts[i].mps);
  if (mscIf >= 0 && mscIn && mscOut) { logf("disk: if %d in %02x out %02x mps %u", mscIf, mscIn, mscOut, mscMps); diskReady = true; return; }
  if (vsIf < 0 || !isoEp || !fmtIndex || !frameIndex) { logf("no usable MJPEG stream on this device"); return; }
  camReady = true;
  if (vcIf >= 0 && intrEp) {
    esp_err_t e = usb_host_interface_claim(client, dev, vcIf, 0);
    logf("claim control interface: 0x%x", e);
    if (e == ESP_OK) {
      intrX->device_handle = dev; intrX->bEndpointAddress = intrEp; intrX->callback = intrCb;
      intrX->num_bytes = 64;
      logf("status listener submit: 0x%x", usb_host_transfer_submit(intrX));
    }
  }
}

static void finishFrame(bool eof) {
  bool ok = eof && !frameErr && frameLen > 64 && frameBuf[0] == 0xFF && frameBuf[1] == 0xD8;
  if (ok) {
    goodFrames++;
    memcpy(keepBuf, frameBuf, frameLen); keepLen = frameLen;
    if (goodFrames >= WANT_FRAMES) captured = true;
  } else if (frameLen) badFrames++;
  frameLen = 0; frameErr = false;
}

static void isoCb(usb_transfer_t *t) {
  if (streaming && !captured) {
    for (int i = 0; i < t->num_isoc_packets; i++) {
      const usb_isoc_packet_desc_t *d = &t->isoc_packet_desc[i];
      const uint8_t *p = t->data_buffer + i * isoMps;
      int n = d->actual_num_bytes;
      if (d->status != USB_TRANSFER_STATUS_COMPLETED) { pktErr++; continue; }
      if (n < 2) continue;
      pktOk++;
      int hl = p[0]; uint8_t inf = p[1];
      if (dDumped < 6) { dDumped++; char t[24]; snprintf(t, sizeof t, "pkt n=%d", n); logHex(t, p, n < 24 ? n : 24); }
      if (hl < 2 || hl > n) { dBadHdr++; continue; }
      if (n > hl) {
        dPayloadPkts++; dPayloadBytes += n - hl;
        if (dDumpedData < 6) { dDumpedData++; char t[24]; snprintf(t, sizeof t, "data n=%d", n); logHex(t, p, n < 32 ? n : 32); }
      } else dHdrOnly++;
      if (inf & 0x02) dEof++;
      int fid = inf & 1;
      if (fid != lastFid) { if (lastFid >= 0) dFidToggles++; if (frameLen) finishFrame(false); lastFid = fid; }
      if (inf & 0x40) frameErr = true;
      int m = n - hl;
      if (m > 0) { if (frameLen + m <= FRAME_CAP) { memcpy(frameBuf + frameLen, p + hl, m); frameLen += m; } else frameErr = true; }
      if (inf & 0x02) finishFrame(true);
    }
  }
  if (streaming && !captured && t->status != USB_TRANSFER_STATUS_NO_DEVICE) {
    if (usb_host_transfer_submit(t) == ESP_OK) return;
  }
  isoPending--;
}

// Negotiate, stream until a few good frames have arrived, stop. Result in keepBuf/keepLen.
static bool captureFrame() {
  g_why[0] = 0;
  uint8_t probe[34];
  int plen = bcdUVC >= 0x0110 ? 34 : 26;
  memset(probe, 0, sizeof probe);
  probe[0] = 1; probe[2] = fmtIndex; probe[3] = frameIndex; put32(probe + 4, frameInterval);
  int r = ctrl(0x21, 0x01, 0x0100, vsIf, probe, plen);
  if (r < 0 && plen == 34) { plen = 26; r = ctrl(0x21, 0x01, 0x0100, vsIf, probe, plen); }
  if (r < 0) { WHY("probe set failed (%d)", r); return false; }
  r = ctrl(0xA1, 0x81, 0x0100, vsIf, probe, plen);
  if (r < 0) { WHY("probe get failed (%d)", r); return false; }
  logHex("probe", probe, plen);
  uint32_t maxFrame = le32(probe + 18), maxPayload = le32(probe + 22);
  logf("negotiated fmt %u frame %u interval %lu maxFrame %lu maxPayload %lu", probe[2], probe[3],
       (unsigned long)le32(probe + 4), (unsigned long)maxFrame, (unsigned long)maxPayload);
  if (ctrl(0x21, 0x01, 0x0200, vsIf, probe, plen) < 0) { WHY("commit failed"); return false; }

  // The camera may only start its video pipeline on the alt setting a
  // high-speed host would pick, so try the highest-numbered one first.
  for (int a = nAlts - 1; a >= 0; a--) {
    if (alts[a].mps < maxPayload && a != 0) continue;
    if (streamOnAlt(a)) return true;
    if (devGone) break;
    uint8_t ec = 0xEE;
    int r2 = ctrl(0xA1, 0x81, 0x0600, vsIf, &ec, 1);          // VS stream error code
    logf("alt %u gave no frame; stream error code %d (r=%d)", alts[a].alt, ec, r2);
  }
  return false;
}

static bool streamOnAlt(int pick) {
  isoMps = alts[pick].mps;
  esp_err_t e = usb_host_interface_claim(client, dev, vsIf, alts[pick].alt);
  logf("claim stream interface alt %u mps %u: 0x%x", alts[pick].alt, isoMps, e);
  if (e != ESP_OK) { WHY("claim alt %u mps %u failed 0x%x", alts[pick].alt, isoMps, e); return false; }
  bool ok = false;
  if (ctrl(0x01, 0x0B, alts[pick].alt, vsIf, nullptr, 0) < 0) WHY("set interface failed");
  else {
    frameLen = keepLen = 0; lastFid = -1; goodFrames = badFrames = pktErr = pktOk = 0; frameErr = false; captured = false;
    dHdrOnly = dPayloadPkts = dEof = dFidToggles = dBadHdr = dDumped = dDumpedData = 0; dPayloadBytes = 0;
    streaming = true; isoPending = 0;
    for (int i = 0; i < ISO_XFERS; i++) {
      usb_transfer_t *t = isoX[i];
      t->device_handle = dev; t->bEndpointAddress = isoEp; t->callback = isoCb;
      t->num_bytes = ISO_PKTS * isoMps;
      for (int k = 0; k < ISO_PKTS; k++) t->isoc_packet_desc[k].num_bytes = isoMps;
      e = usb_host_transfer_submit(t);
      if (e == ESP_OK) isoPending++; else WHY("iso submit %d failed 0x%x", i, e);
    }
    uint32_t t0 = millis();
    while (!captured && isoPending > 0 && !devGone && millis() - t0 < 5000)
      usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
    streaming = false;
    uint32_t t1 = millis();
    while (isoPending > 0 && millis() - t1 < 1000) usb_host_client_handle_events(client, pdMS_TO_TICKS(10));
    logf("alt %u %lu ms: good %d bad %d pktOk %d pktErr %d kept %u", alts[pick].alt, (unsigned long)(t1 - t0),
         goodFrames, badFrames, pktOk, pktErr, (unsigned)keepLen);
    logf(" header-only %d, with data %d (%lu bytes), end-of-frame flags %d, frame toggles %d, bad headers %d",
         dHdrOnly, dPayloadPkts, (unsigned long)dPayloadBytes, dEof, dFidToggles, dBadHdr);
    ok = keepLen > 0;
    if (!ok) WHY("alt %u: frames %d/%d, pkts %d, data pkts %d, %lu bytes, eof %d", alts[pick].alt, goodFrames, badFrames,
                 pktOk, dPayloadPkts, (unsigned long)dPayloadBytes, dEof);
    if (!devGone) ctrl(0x01, 0x0B, 0, vsIf, nullptr, 0);
  }
  if (isoPending > 0) { usb_host_endpoint_halt(dev, isoEp); usb_host_endpoint_flush(dev, isoEp); }
  e = usb_host_interface_release(client, dev, vsIf);
  if (e != ESP_OK) logf("release stream interface: 0x%x", e);
  return ok;
}

// ---------------------------------------------------------------- camera card as a USB disk
static usb_transfer_t *bulkX;
static volatile bool bulkDone;
static void bulkCb(usb_transfer_t *) { bulkDone = true; }
#define BULK_BUF (16 * 512)

// One bulk transfer; returns bytes moved or <0. IN lengths are rounded up to the packet size.
static int bulk(uint8_t ep, uint8_t *data, int len) {
  bool in = ep & 0x80;
  int req = in ? ((len + mscMps - 1) / mscMps) * mscMps : len;
  if (req > BULK_BUF + 64) return -9;
  if (!in) memcpy(bulkX->data_buffer, data, len);
  bulkX->num_bytes = req; bulkX->device_handle = dev; bulkX->bEndpointAddress = ep; bulkX->callback = bulkCb;
  bulkDone = false;
  esp_err_t e = usb_host_transfer_submit(bulkX);
  if (e != ESP_OK) { logf("bulk submit ep %02x: 0x%x", ep, e); return -1; }
  uint32_t t0 = millis();
  while (!bulkDone && !devGone && millis() - t0 < 4000) usb_host_client_handle_events(client, pdMS_TO_TICKS(5));
  if (!bulkDone) {
    usb_host_endpoint_halt(dev, ep); usb_host_endpoint_flush(dev, ep);
    uint32_t t1 = millis();
    while (!bulkDone && millis() - t1 < 500) usb_host_client_handle_events(client, pdMS_TO_TICKS(5));
    usb_host_endpoint_clear(dev, ep);
    return -2;
  }
  if (bulkX->status == USB_TRANSFER_STATUS_STALL) {
    usb_host_endpoint_clear(dev, ep);
    ctrl(0x02, 0x01, 0, ep, nullptr, 0);               // CLEAR_FEATURE(ENDPOINT_HALT)
    return -4;
  }
  if (bulkX->status != USB_TRANSFER_STATUS_COMPLETED) return -3;
  int n = bulkX->actual_num_bytes;
  if (in && n > 0) memcpy(data, bulkX->data_buffer, n < len ? n : len);
  return n;
}

static uint32_t mscTag = 1, diskBlocks = 0, diskBlockSize = 512;
// SCSI command over bulk-only transport. Returns 0 on success.
static int scsi(const uint8_t *cdb, int cdbLen, uint8_t *data, int dataLen) {
  uint8_t cbw[31] = {'U', 'S', 'B', 'C'};
  put32(cbw + 4, mscTag++); put32(cbw + 8, dataLen);
  cbw[12] = 0x80; cbw[13] = 0; cbw[14] = cdbLen;
  memcpy(cbw + 15, cdb, cdbLen);
  int r = bulk(mscOut, cbw, 31);
  if (r != 31) { logf("scsi %02x: command stage %d", cdb[0], r); return -1; }
  if (dataLen) {
    r = bulk(mscIn, data, dataLen);
    if (r != dataLen && r != -4) { logf("scsi %02x: data stage %d of %d", cdb[0], r, dataLen); if (r < 0) return -2; }
  }
  uint8_t csw[64];
  r = bulk(mscIn, csw, 13);
  if (r == -4) r = bulk(mscIn, csw, 13);
  if (r < 13 || memcmp(csw, "USBS", 4)) { logf("scsi %02x: status stage %d", cdb[0], r); return -3; }
  return csw[12];
}

static bool diskStart() {
  esp_err_t e = usb_host_interface_claim(client, dev, mscIf, 0);
  if (e != ESP_OK) { logf("claim disk interface: 0x%x", e); return false; }
  uint8_t tur[6] = {0}, sense[18];
  uint8_t rs[6] = {0x03, 0, 0, 0, 18, 0};
  bool ready = false;
  for (int i = 0; i < 20 && !devGone; i++) {
    if (scsi(tur, 6, nullptr, 0) == 0) { ready = true; break; }
    scsi(rs, 6, sense, 18);
    delay(100);
  }
  if (!ready) { logf("disk never became ready"); return false; }
  uint8_t rc[10] = {0x25}, cap[8];
  if (scsi(rc, 10, cap, 8) != 0) { logf("read capacity failed"); return false; }
  diskBlocks = ((uint32_t)cap[0] << 24 | cap[1] << 16 | cap[2] << 8 | cap[3]) + 1;
  diskBlockSize = (uint32_t)cap[4] << 24 | cap[5] << 16 | cap[6] << 8 | cap[7];
  logf("disk %lu blocks of %lu bytes", (unsigned long)diskBlocks, (unsigned long)diskBlockSize);
  uint8_t inq[6] = {0x12, 0, 0, 0, 36, 0}, inqData[36] = {0};
  if (scsi(inq, 6, inqData, 36) == 0) logHex("inquiry", inqData, 36);   // carries the camera patch's diagnostics
  return diskBlockSize == 512;
}

static DSTATUS dkInit(unsigned char) { return 0; }
static DSTATUS dkStatus(unsigned char) { return 0; }
static DRESULT dkRead(unsigned char, unsigned char *buff, uint32_t sector, unsigned count) {
  while (count) {
    unsigned n = count > 16 ? 16 : count;
    uint8_t cdb[10] = {0x28, 0, (uint8_t)(sector >> 24), (uint8_t)(sector >> 16), (uint8_t)(sector >> 8), (uint8_t)sector, 0, (uint8_t)(n >> 8), (uint8_t)n, 0};
    if (scsi(cdb, 10, buff, n * 512) != 0) return RES_ERROR;
    buff += n * 512; sector += n; count -= n;
  }
  return RES_OK;
}
static DRESULT dkWrite(unsigned char, const unsigned char *, uint32_t, unsigned) { return RES_WRPRT; }
static DRESULT dkIoctl(unsigned char, unsigned char cmd, void *buff) {
  switch (cmd) {
    case CTRL_SYNC: return RES_OK;
    case GET_SECTOR_COUNT: *(LBA_t *)buff = diskBlocks; return RES_OK;
    case GET_SECTOR_SIZE: *(WORD *)buff = 512; return RES_OK;
    case GET_BLOCK_SIZE: *(DWORD *)buff = 1; return RES_OK;
  }
  return RES_PARERR;
}
static const ff_diskio_impl_t dkImpl = {dkInit, dkStatus, dkRead, dkWrite, dkIoctl};
static int dkDrive = -1;
static char lastPhoto[20];
static uint32_t lastPhotoSize = 0;

// Mount the camera's card, find the highest-numbered JPEG in DCIM and read it
// into keepBuf. Returns 1 = new photo read, 0 = nothing new, <0 = failed.
static int pullPhoto() {
  g_why[0] = 0;
  uint32_t t0 = millis();
  if (!diskStart()) { WHY("card did not answer as a USB disk"); return -1; }
  if (dkDrive < 0) {
    BYTE pdrv;
    if (ff_diskio_get_drive(&pdrv) != ESP_OK) { WHY("no free FAT drive slot"); return -1; }
    dkDrive = pdrv;
  }
  ff_diskio_register(dkDrive, &dkImpl);
  char root[4] = {(char)('0' + dkDrive), ':', 0, 0};
  static FATFS *fs;
  if (!fs) fs = (FATFS *)ps_malloc(sizeof(FATFS));
  FRESULT fr = f_mount(fs, root, 1);
  if (fr != FR_OK) { WHY("could not mount the card (FAT error %d; exFAT cards are not supported)", fr); return -1; }
  char dir[16]; snprintf(dir, sizeof dir, "%sDCIM", root);
  FF_DIR d; FILINFO fi;
  char best[20] = ""; uint32_t bestSize = 0; int nFiles = 0;
  fr = f_opendir(&d, dir);
  if (fr != FR_OK) { WHY("no DCIM folder on the card (FAT error %d)", fr); f_mount(nullptr, root, 0); return -1; }
  while (f_readdir(&d, &fi) == FR_OK && fi.fname[0]) {
    if (fi.fattrib & AM_DIR) continue;
    size_t l = strlen(fi.fname);
    if (l < 5 || l >= sizeof best || strcasecmp(fi.fname + l - 4, ".jpg")) continue;
    nFiles++;
    if (strcasecmp(fi.fname, best) > 0) { strcpy(best, fi.fname); bestSize = fi.fsize; }
  }
  f_closedir(&d);
  logf("card: %d photos, newest %s (%lu bytes)", nFiles, best, (unsigned long)bestSize);
  int result = 0;
  if (!best[0]) WHY("no photos on the card");
  else if (!strcasecmp(best, lastPhoto) && bestSize == lastPhotoSize) WHY("%s was already printed", best);
  else if (bestSize > FRAME_CAP) { WHY("%s is too large (%lu bytes)", best, (unsigned long)bestSize); result = -1; }
  else {
    char path[40]; snprintf(path, sizeof path, "%s/%s", dir, best);
    FIL f; UINT got = 0;
    fr = f_open(&f, path, FA_READ);
    if (fr == FR_OK) { fr = f_read(&f, keepBuf, bestSize, &got); f_close(&f); }
    if (fr != FR_OK || got != bestSize) { WHY("reading %s failed (FAT error %d, %u of %lu bytes)", best, fr, got, (unsigned long)bestSize); result = -1; }
    else { keepLen = got; strcpy(lastPhoto, best); lastPhotoSize = bestSize; result = 1; }
  }
  f_mount(nullptr, root, 0);
  logf("card session %lu ms, result %d", (unsigned long)(millis() - t0), result);
  return result;
}

// ---------------------------------------------------------------- image -> 1-bit raster
static M5Canvas canvas;
static uint8_t *job; static size_t jobLen = 0, jobCap = 0;
static tdefl_compressor *deflator;

static bool jpegSize(const uint8_t *p, size_t n, int *w, int *h, bool *hasDht) {
  *hasDht = false;
  bool found = false;
  size_t i = 2;
  while (i + 9 <= n && p[i] == 0xFF) {
    uint8_t m = p[i + 1];
    size_t l = (p[i + 2] << 8) | p[i + 3];
    if (m == 0xC4) *hasDht = true;
    if (m == 0xC0 || m == 0xC1 || m == 0xC2) { *h = (p[i + 5] << 8) | p[i + 6]; *w = (p[i + 7] << 8) | p[i + 8]; found = true; }
    if (m == 0xDA) break;
    i += 2 + l;
  }
  return found;
}

static void jobAppend(const void *p, size_t l) { if (jobLen + l <= jobCap) { memcpy(job + jobLen, p, l); jobLen += l; } }

// JPEG decoding: bitbank2's JPEGDEC (ESP32-S3 SIMD paths) straight to 8-bit
// grey, at 1/2 or 1/4 scale. Falls back to the display library's decoder.
static JPEGDEC *g_jdec;
static uint8_t *g_gray; static int g_gw, g_gh;
static int jdecDraw(JPEGDRAW *d) {
  const uint8_t *src = (const uint8_t *)d->pPixels;
  int w = d->iWidthUsed;
  if (d->x + w > g_gw) w = g_gw - d->x;
  for (int r = 0; r < d->iHeight && d->y + r < g_gh; r++)
    if (w > 0) memcpy(g_gray + (size_t)(d->y + r) * g_gw + d->x, src + (size_t)r * d->iWidth, w);
  return 1;
}

// 0 = Floyd-Steinberg error diffusion, 1 = Bayer 4x4, 2 = Bayer 8x8 (default:
// ordered dither compresses to about a third, so the Bluetooth send is ~1 s
// shorter). Changed by holding the print-again key, or over serial with
// 'm0'/'m1'/'m2'; kept in flash.
static int g_dither = 2;
static const uint8_t BAYER4[4][4] = {{0, 8, 2, 10}, {12, 4, 14, 6}, {3, 11, 1, 9}, {15, 7, 13, 5}};
static const uint8_t BAYER8[8][8] = {
  {0, 32, 8, 40, 2, 34, 10, 42}, {48, 16, 56, 24, 50, 18, 58, 26}, {12, 44, 4, 36, 14, 46, 6, 38},
  {60, 28, 52, 20, 62, 30, 54, 22}, {3, 35, 11, 43, 1, 33, 9, 41}, {51, 19, 59, 27, 49, 17, 57, 25},
  {15, 47, 7, 39, 13, 45, 5, 37}, {63, 31, 55, 23, 61, 29, 53, 21}};
static const char *ditherName() { return g_dither == 1 ? "Bayer 4x4" : g_dither == 2 ? "Bayer 8x8" : "error diffusion"; }
static void setDither(int d) {
  g_dither = d;
  if (g_fat) { File f = FFat.open("/dither.txt", "w"); if (f) { f.print(d); f.close(); } }
}

// The current photo as a 1-bit image W dots wide: 1 = black, leftmost dot in
// the top bit of each byte, rows of (W + 7) / 8 bytes. Height keeps the photo's
// aspect, capped at maxRows. The printer driver turns it into its own job.
static uint8_t *g_rast; static int g_rw, g_rh, g_rbpr;
static bool renderRaster(const uint8_t *jpg, size_t len, int HEAD, int maxRows) {
  const int BPR = (HEAD + 7) / 8;
  int w = 0, h = 0; bool dht = false;
  if (!jpegSize(jpg, len, &w, &h, &dht) || w < 16 || h < 16) { logf("jpeg header not understood"); logHex("jpg", jpg, len < 96 ? len : 96); return false; }
  logf("jpeg %dx%d %u bytes, huffman tables %s", w, h, (unsigned)len, dht ? "present" : "MISSING");
  int H = (int)((int64_t)h * HEAD / w);
  if (H > maxRows) H = maxRows;
  // Decode at exactly 1/2 (or 1/4 for big images): the decoder then skips work
  // inside the IDCT and takes its fast, non-resampling path. The resample to
  // 576 dots wide happens in the dither loop below.
  int div = w >= 4 * HEAD ? 4 : w >= 2 * HEAD ? 2 : 1;
  int SW = w / div, SH = h / div;
  uint8_t *px = nullptr;
  static uint8_t *grayBuf; static size_t grayCap;
  size_t need = (size_t)(SW + 16) * (SH + 16);
  if (need > grayCap) { free(grayBuf); grayBuf = (uint8_t *)ps_malloc(need); grayCap = grayBuf ? need : 0; }
  if (!g_jdec) g_jdec = new JPEGDEC();
  if (grayBuf && g_jdec->openRAM((uint8_t *)jpg, len, jdecDraw)) {
    g_jdec->setPixelType(EIGHT_BIT_GRAYSCALE);
    g_gray = grayBuf; g_gw = SW; g_gh = SH;
    memset(grayBuf, 0xff, (size_t)SW * SH);
    int opt = div == 4 ? JPEG_SCALE_QUARTER : div == 2 ? JPEG_SCALE_HALF : 0;
    if (g_jdec->decode(0, 0, opt)) px = grayBuf;
    else logf("JPEGDEC failed (%d), using the fallback decoder", g_jdec->getLastError());
    g_jdec->close();
  }
  if (!px) {
    float dsc = 1.0f / div;
    canvas.deleteSprite();
    canvas.setPsram(true);
    canvas.setColorDepth(lgfx::grayscale_8bit);
    if (!canvas.createSprite(SW, SH)) { logf("sprite alloc failed"); return false; }
    canvas.fillSprite(TFT_WHITE);
    if (!canvas.drawJpg(jpg, len, 0, 0, SW, SH, 0, 0, dsc, dsc)) { logf("jpeg decode failed"); return false; }
    px = (uint8_t *)canvas.getBuffer();
  }
  logf("decoded at %dx%d", SW, SH);

  // stretch contrast between the 1st and 99th percentile
  uint32_t hist[256] = {0};
  for (int i = 0; i < SW * SH; i++) hist[px[i]]++;
  uint32_t cut = (uint32_t)(SW * SH) / 100, acc = 0; int lo = 0, hi = 255;
  for (lo = 0; lo < 255; lo++) { acc += hist[lo]; if (acc > cut) break; }
  acc = 0;
  for (hi = 255; hi > 0; hi--) { acc += hist[hi]; if (acc > cut) break; }
  if (hi - lo < 16) { lo = 0; hi = 255; }
  logf("levels %d..%d", lo, hi);

  // Floyd-Steinberg, two rows of error
  int16_t *err = (int16_t *)calloc((HEAD + 2) * 2, sizeof(int16_t));
  static uint8_t *rastBuf; static size_t rastCap;
  if ((size_t)BPR * H > rastCap) { free(rastBuf); rastBuf = (uint8_t *)ps_malloc((size_t)BPR * H); rastCap = rastBuf ? (size_t)BPR * H : 0; }
  uint8_t *bmp = rastBuf;
  if (!err || !bmp) { logf("dither alloc failed"); free(err); return false; }
  memset(bmp, 0x00, BPR * H);
  int16_t *e0 = err + 1, *e1 = err + HEAD + 3;
  static uint16_t xmap[1024];
  if (HEAD > 1024) { free(err); return false; }
  for (int x = 0; x < HEAD; x++) xmap[x] = (uint16_t)((uint32_t)x * SW / HEAD);
  int span = hi - lo;
  for (int y = 0; y < H && g_dither != 0; y++) {
    // Ordered (Bayer) dither: a fixed repeating threshold pattern. The output
    // repeats every 4 or 8 pixels, so the job compresses far better than error
    // diffusion's noise and the Bluetooth send is shorter.
    const uint8_t *row = px + (size_t)((uint32_t)y * SH / H) * SW;
    uint8_t *out = bmp + y * BPR;
    for (int x = 0; x < HEAD; x++) {
      int v = ((int)row[xmap[x]] - lo) * 255 / span;
      int t = g_dither == 1 ? BAYER4[y & 3][x & 3] * 16 + 8 : BAYER8[y & 7][x & 7] * 4 + 2;
      if (v < t) out[x >> 3] |= 0x80 >> (x & 7);
    }
  }
  for (int y = 0; y < H && g_dither == 0; y++) {
    memset(e1 - 1, 0, (HEAD + 2) * sizeof(int16_t));
    const uint8_t *row = px + (size_t)((uint32_t)y * SH / H) * SW;
    for (int x = 0; x < HEAD; x++) {
      int v = ((int)row[xmap[x]] - lo) * 255 / span;
      v = (v < 0 ? 0 : v > 255 ? 255 : v) + e0[x] / 16;
      int nw = v >= 128 ? 255 : 0;
      if (!nw) bmp[y * BPR + (x >> 3)] |= 0x80 >> (x & 7);
      int d = v - nw;
      e0[x + 1] += d * 7; e1[x - 1] += d * 3; e1[x] += d * 5; e1[x + 1] += d;
    }
    int16_t *t = e0; e0 = e1; e1 = t;
  }
  free(err);

  g_rast = bmp; g_rw = HEAD; g_rh = H; g_rbpr = BPR;
  logf("raster %dx%d (%s)", HEAD, H, ditherName());
  return true;
}

// Render the photo for the connected printer (576 dots if none yet) and build
// its job.
static bool buildJob(const uint8_t *jpg, size_t len) {
  if (!renderRaster(jpg, len, printerWidth(), printerMaxRows())) return false;
  return printerBuild(g_rast, g_rw, g_rh);
}

// ---------------------------------------------------------------- flow
static bool g_vbusLow = false;   // port had no 5 V when host mode started
// Set while diagnosing the camera: printing and the screen refresh take ~40 s,
// during which the bridge would miss the camera's next USB window.
static const bool DIAG_NO_PRINT = false;
static int shots = 0;
static void finish(const String &line, bool good) {
  logf("%s", line.c_str());
  saveLog();
  beep(good ? 2000 : 300, good ? 150 : 600);
  if (!good || !g_epaper) screen(g_status + "\n" + line);   // e-ink success: skip the multi-second refresh
}

static void doPrint() {
  uint32_t t0 = millis();
  const char *err = printerSend();
  if (err) finish(String("Print failed: ") + err, false);
  else finish(String("Printed, ") + jobLen + " bytes in " + (millis() - t0) / 1000 + " s", true);
}

// The camera has just presented its card: pull the newest photo and print it.
static void doCardPhoto() {
  shots++;
  logf("--- card photo %d", shots);
  // Read the photo and go quiet. The camera ends its own USB window a few
  // seconds later (patch/shotsync.s) and drops off the bus by itself; polling it
  // meanwhile only left a transfer stuck when it stopped answering.
  int r = pullPhoto();
  if (r < 0) { finish(String("Could not read the photo:\n") + g_why, false); return; }
  if (r == 0) { logf("nothing to print"); saveLog(); return; }
  beep(1200, 60);
  if (DIAG_NO_PRINT) { logf("diagnostic build: not printing %s", lastPhoto); saveLog(); return; }
  // Printer first: the image is rendered at the width of whichever printer is
  // there (384, 568 or 576 dots).
  const char *perr = printerEnsure();
  if (perr) { finish(String(lastPhoto) + ": " + perr, false); return; }
  if (!buildJob(keepBuf, keepLen)) { finish(String("Could not turn ") + lastPhoto + " into a print job", false); return; }
  uint32_t t0 = millis();
  const char *err = printerSend();
  if (err) finish(String(lastPhoto) + ": print failed, " + err, false);
  else finish(String("Printed ") + lastPhoto + " in " + (millis() - t0) / 1000 + " s", true);
}

static void doShot() {
  shots++;
  logf("--- shot %d", shots);
  beep(1200, 60);
  if (!camReady) { finish("No camera attached", false); return; }
  if (!captureFrame()) { finish(String("No frame from the camera:\n") + g_why, false); return; }
  if (!buildJob(keepBuf, keepLen)) { finish("Could not turn the frame into a print job", false); return; }
  doPrint();
}

// The internal USB PHY selection lives in the RTC domain and survives a
// software restart, so give it back to the USB serial function explicitly.
static void phyToSerial() { CLEAR_PERI_REG_MASK(RTC_CNTL_USB_CONF_REG, RTC_CNTL_SW_HW_USB_PHY_SEL | RTC_CNTL_SW_USB_PHY_SEL); }
static void restartTo(bool serial) { g_forceSerial = serial ? 1 : 0; saveLog(); phyToSerial(); ESP.restart(); }

static int vbusMv() { return M5.Power.getVBUSVoltage(); }

static void serialLoop() {
  bool wasPlugged = false; uint32_t unpluggedAt = 0;
  for (;;) {
    M5.update();
    bool plugged = HWCDC::isPlugged();
    if (plugged) { wasPlugged = true; unpluggedAt = 0; }
    else if (wasPlugged) { if (!unpluggedAt) unpluggedAt = millis(); else if (millis() - unpluggedAt > 3000) restartTo(false); }
    if ((g_stick ? M5.BtnB : M5.BtnA).wasPressed()) restartTo(false);
    while (Serial.available()) {
      int c = Serial.read();
      if (c == 'd') {
        Serial.printf("\n=== bridge log, %lu bytes ===\n", (unsigned long)g_logLen);
        for (uint32_t i = 0; i < g_logLen; i += 256) { Serial.write((const uint8_t *)g_log + i, g_logLen - i < 256 ? g_logLen - i : 256); Serial.flush(); }
        Serial.println("=== end ===");
      } else if (c == 'c') { g_logLen = 0; Serial.println("log cleared"); }
      else if (c == 'j') {
        // test path: 'j' + 4-byte little-endian length + a JPEG -> builds the print job and returns it
        uint8_t hdr[4]; size_t got = 0; uint32_t t0 = millis();
        while (got < 4 && millis() - t0 < 3000) if (Serial.available()) hdr[got++] = Serial.read();
        uint32_t len = le32(hdr); got = 0; t0 = millis();
        if (len == 0 || len > FRAME_CAP) { Serial.println("JOBFAIL bad length"); continue; }
        while (got < len && millis() - t0 < 20000) { int a = Serial.available(); if (a > 0) got += Serial.read(keepBuf + got, min((size_t)a, (size_t)(len - got))); else delay(1); }
        keepLen = got;
        if (!printerReady()) printerEnsure();       // render for the printer that is there
        if (got != len || !buildJob(keepBuf, keepLen)) { Serial.printf("JOBFAIL got %u of %u\n", (unsigned)got, (unsigned)len); continue; }
        Serial.printf("JOB %u\n", (unsigned)jobLen);
        for (size_t i = 0; i < jobLen; i += 256) { Serial.write(job + i, jobLen - i < 256 ? jobLen - i : 256); Serial.flush(); }
        Serial.println("\nJOBEND");
      } else if (c == 's') {
        printerList();
      } else if (c == 'n') {
        // prefer the printer whose name contains the text up to the newline (testing)
        String frag; uint32_t t0 = millis();
        while (millis() - t0 < 2000) { if (Serial.available()) { char ch = Serial.read(); if (ch == '\n') break; frag += ch; } }
        printerPrefer(frag);
        Serial.printf("PREFER %s\n", frag.c_str());
      } else if (c == 'm') {
        uint32_t t0 = millis(); int d = -1;
        while (d < 0 && millis() - t0 < 2000) if (Serial.available()) d = Serial.read() - '0';
        if (d >= 0 && d <= 2) setDither(d);
        Serial.printf("DITHER %d %s\n", g_dither, ditherName());
      } else if (c == 'v') {
        Serial.printf("VBUS %d mV, battery %d%%, charging %d\n", vbusMv(), (int)M5.Power.getBatteryLevel(), (int)M5.Power.isCharging());
      } else if (c == 'p') {
        // test path: send the current job to the printer
        const char *err = printerSend();
        Serial.printf("PRINT %s\n", err ? err : "ok");
      }
    }
    delay(20);
  }
}

void setup() {
  phyToSerial();
  auto cfg = M5.config();
  M5.begin(cfg);
  g_epaper = M5.getBoard() == m5::board_t::board_M5PaperColor;
  g_stick = M5.getBoard() == m5::board_t::board_M5StickS3;
  if (g_stick) M5.Display.setRotation(1);   // landscape, 240x135
  M5.Speaker.setVolume(160);
  Serial.setRxBufferSize(16384);
  Serial.begin(115200);
  g_fat = FFat.begin(true);
  if (g_fat) { File f = FFat.open("/dither.txt", "r"); if (f) { int d = f.read() - '0'; if (d >= 0 && d <= 2) g_dither = d; f.close(); } }
  if (g_logMagic != LOG_MAGIC || g_logLen >= LOG_CAP) { g_logMagic = LOG_MAGIC; g_logLen = 0; g_forceSerial = 0; loadLog(); }

  frameBuf = (uint8_t *)ps_malloc(FRAME_CAP);
  keepBuf = (uint8_t *)ps_malloc(FRAME_CAP);
  jobCap = 256 * 1024; job = (uint8_t *)ps_malloc(jobCap);
  deflator = (tdefl_compressor *)ps_malloc(sizeof(tdefl_compressor));
  if (!frameBuf || !keepBuf || !job || !deflator) { logf("buffer alloc failed"); screen("Out of memory at start"); return; }
  NimBLEDevice::init("charmera-bridge");
  NimBLEDevice::setMTU(247);

  bool force = g_forceSerial == 1;
  g_forceSerial = 0;
  bool plugged = false;
  for (int i = 0; i < 30 && !plugged; i++) { delay(100); plugged = HWCDC::isPlugged(); }
  if (plugged || force) {
    g_serialMode = true;
    screen("Serial mode\n\nPlugged into a computer.\nUnplug to return to camera mode.");
    serialLoop();
  }

  g_logLen = 0;                                         // new host session, new log
  logf("bridge start, psram free %u", (unsigned)ESP.getFreePsram());

  usb_host_config_t hostCfg = {};
  hostCfg.intr_flags = ESP_INTR_FLAG_LEVEL1;
  hostCfg.fifo_settings_custom.nptx_fifo_lines = 40;    // room for 512-byte isochronous IN packets
  hostCfg.fifo_settings_custom.ptx_fifo_lines = 16;
  hostCfg.fifo_settings_custom.rx_fifo_lines = 144;
  esp_err_t e = usb_host_install(&hostCfg);
  logf("usb_host_install: 0x%x", e);
  if (e != ESP_OK) { screen("USB host failed to start"); return; }
  xTaskCreatePinnedToCore(libTask, "usbh", 4096, nullptr, 5, nullptr, 0);
  usb_host_client_config_t cc = {};
  cc.is_synchronous = false;
  cc.max_num_event_msg = 8;
  cc.async.client_event_callback = clientCb;
  usb_host_client_register(&cc, &client);
  usb_host_transfer_alloc(8 + 256, 0, &ctrlX);
  usb_host_transfer_alloc(64, 0, &intrX);
  usb_host_transfer_alloc(BULK_BUF + 64, 0, &bulkX);
  for (int i = 0; i < ISO_XFERS; i++) usb_host_transfer_alloc(ISO_PKTS * 1024, ISO_PKTS, &isoX[i]);

  // "Plugged into a computer" detection. The PaperColor's power chip reports the
  // USB-C voltage; the StickS3's reads ~5 V even on battery, so it uses the
  // charging state instead (the camera never supplies power, so charging means
  // a computer or charger is on the port).
  g_vbusLow = g_stick ? M5.Power.isCharging() != m5::Power_Class::is_charging : vbusMv() < 3000;
  logf("vbus at start %d mV, charging %d", vbusMv(), (int)M5.Power.isCharging());
  screen("Charmera print bridge\n\nConnecting to the printer...");
  const char *berr = printerEnsure();
  if (berr) logf("printer: %s (will look again at the first print)", berr);
  g_status = g_stick ? "Charmera print bridge\nFront: print again\n  (hold: dither mode)\nSide: computer mode\n"
                     : "Charmera print bridge\n\nUp: computer mode\nDown: photo + print\nThird key: print again\n  (hold: dither mode)\n";
  g_status += String("Dither: ") + ditherName() + "\n";
  g_status += String("Printer: ") + printerName() + "\n";
  screen(g_status + "\nReady. Take a photo on the camera.");
}

void loop() {
  if (!client) { delay(1000); return; }
  M5.update();
  usb_host_client_handle_events(client, pdMS_TO_TICKS(20));
  if (newAddr) {
    uint8_t a = newAddr; newAddr = 0;
    openCamera(a);
    if (diskReady) doCardPhoto();                       // no screen update first: the camera's window is short
    else {
      beep(camReady ? 1600 : 300, 120);
      saveLog();
      screen(g_status + (camReady ? "\nCamera connected (webcam mode)." : "\nUSB device is not a usable camera."));
    }
  }
  if (devGone) {
    devGone = false;
    bool wasDisk = diskReady;
    camReady = diskReady = false;
    if (dev) {
      if (wasDisk) usb_host_interface_release(client, dev, mscIf);
      else if (vcIf >= 0) usb_host_interface_release(client, dev, vcIf);
      usb_host_device_close(client, dev); dev = nullptr;
    }
    logf("camera gone");
    if (!wasDisk) screen(g_status + "\nCamera disconnected.");
  }
  if ((g_stick ? M5.BtnB : M5.BtnA).wasPressed()) restartTo(true);
  // Cable moved from the camera to a computer: 5 V appears on the port.
  static uint32_t lastV = 0; static int highV = 0;
  if (g_vbusLow && millis() - lastV > 1000) {
    lastV = millis();
    bool powered = g_stick ? M5.Power.isCharging() == m5::Power_Class::is_charging : vbusMv() > 4200;
    highV = powered ? highV + 1 : 0;
    if (highV >= 3) { logf("power on the port, switching to computer mode"); restartTo(true); }
  }
  if ((!g_stick && M5.BtnB.wasPressed()) || trigger) { trigger = false; doShot(); }
  m5::Button_Class &printKey = g_stick ? M5.BtnA : M5.BtnC;
  if (printKey.wasHold()) {                              // hold: next dither mode
    setDither((g_dither + 1) % 3);
    logf("dither mode: %s", ditherName());
    beep(1800, 60);
    screen(g_status + "\nDither: " + ditherName());
  } else if (printKey.wasClicked()) { logf("--- reprint"); doPrint(); }
}
