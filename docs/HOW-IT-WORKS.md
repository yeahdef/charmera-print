# How it works

## The problem

The Charmera is a USB *device*: it shows up as a webcam or as a disk when plugged into a computer. To get photos out of it without a computer, something has to act as the USB *host*. The ESP32-S3 in an M5Stack StickS3 can do that, but its USB-C port cannot supply 5 V. And an unmodified Charmera only starts USB when it sees 5 V on the cable.

## The camera patch (`patch/shotsync.s`)

The camera runs a Generalplus GPCV-series SoC (ARM7TDMI, µC/OS-II). All addresses below are for the `v012` build; `v006` is the same code a few bytes apart, and the patch assembles for either.

### USB detection is software

Plug detection is not done in hardware. A routine in the peripheral task (VA `0x7a18`) reads the power-management status register (`0xD0000068`, bit 2 = DC-in) through a helper at `0x6d5c`. When DC-in goes from 0 to 1 and stays there for about 16 runs, it posts message `0x8001` to the USB task. That starts a one-second probe: USB comes up and waits for a host to send a SETUP packet. If one does, the state machine gets `0x7012` and the current mode enters USB-disk mode.

So the patch never has to fake 5 V. It only has to make that routine *believe* DC-in is present, at the right moment.

### Shot sync

Five hooks, with about 450 bytes of code placed in lwIP's `dns_recv`. That function is dead code, because this camera never starts its Wi-Fi stack. The patch's variables live in lwIP's `dns_table`, zeroed RAM the camera never touches.

1. **After a photo is saved:** the hook is on photo mode's (state `0x102`) capture-done handler, where the JPEG is written and the file closed. It arms a timer.
2. **Opening the window:** 50 ms later, the DC-in read used by the plug state machine (`bl 0x6d5c` at `0x7a20`) returns "present", and the state machine's debounce counter is preloaded so the probe starts at once. The bridge answers the probe, and the camera presents its SD card as a USB mass-storage disk (VID:PID `1b3f:8301`).
3. **Closing the window:** a hook in the peripheral task loop (`0x87a4`), which runs in every mode, ends it after 4 s. The bridge needs well under a second. If USB was entered, it writes `0x5A` to RTC register `0x83` and calls the stock power-off routine.
4. **Next boot:** the boot power check (`0x60a8`) reads RTC `0x83` (stock uses it for a low-battery marker, `0xA5`). The hook on its DC-in read (`0x60e4`) sees `0x5A`, clears it, and answers "DC present". Otherwise the camera would switch itself off again, because nobody is holding the power key.
5. **End of startup:** startup normally hands over to the mode-select screen (state `0x10d`). On a requested restart the hook at `0x6384` hands over to photo mode (`0x102`) instead.

If no host answers the probe, nothing else happens: the window closes, two clock bits the probe cleared (`0xD0000008` bits 8 and 15) are put back, and the camera carries on.

### Why the camera switches off after each print

Stock firmware never comes *back* from USB-disk mode; it powers off when the cable is pulled. Entering USB mode:

- stops the image sensor's clock (`0x7b590` clears bits in `0xD000000C`);
- changes system clock bits in `0xD0000008`. The firmware has the matching restore at `0x8a4f8`, but nothing ever calls it.

The sensor (an OV9732 on `v012`) is only started once, at boot (`0x2d328`, called from `0x63d0`).

These were tried, and none brought the viewfinder back:

| Attempt | Result |
|---|---|
| Restore the clock bits, then send the stock "disconnected from PC" message (`0x7013`) | viewfinder stays dead |
| The above plus re-running the boot sensor start | viewfinder stays dead |
| Re-run the whole firmware from its reset handler | the sensor and JPEG path work (a photo taken afterwards is a real image), but the live preview stays black |
| A watchdog reset | drops the power-hold, so the camera just switches off |

A hardware reset is the one thing known to bring everything up. The power-hold is only ever set by hardware while the power key is held, so a hardware reset can't keep the camera on. So the patch powers off cleanly and makes the next power-on go straight to the viewfinder. The software-restart experiment is still in the patch behind `--defsym SOFT_RESTART=1`, for anyone who wants to find the missing piece.

### Things that bit us

- State `0x101` is video and `0x102` is photo. The first hook went on the video save path and never fired.
- Free-looking `0xFF` bytes at `0xf573c`, used by earlier community patches, are the flat top of a lookup table.
- The camera's USB task only checks its "stop USB" message between mass-storage commands. A host that keeps polling keeps it busy; the bridge reads the photo and then goes quiet.
- Two sensor variants exist, and the wrong firmware freezes photo mode on "SELECT MODE". The SD updater runs before photo mode, so it still works.

## The bridge (`bridge/charmera_bridge/`)

- **USB host:** ESP-IDF's USB host stack on the StickS3's USB-C port, at full speed (12 Mbit/s). The camera's webcam mode is useless at that speed, because it only sends empty frames, so the bridge uses its disk mode. A minimal bulk-only-transport SCSI client (`TEST UNIT READY`, `READ CAPACITY`, `READ(10)`) under FatFs mounts the card read-only, finds the newest `DCIM/*.jpg` and reads it, in about 0.1 s.
- **Image:** [JPEGDEC](https://github.com/bitbank2/JPEGDEC) decodes at ½ or ¼ scale straight to 8-bit grey, using the ESP32-S3's SIMD paths: 0.25 s for a 1440×1080 photo. The levels are stretched between the 1st and 99th percentile, then the image is resampled to the printer's head width and dithered.
- **Printer discovery:** a 3-second scan keeps only devices it can identify, in this order:
  1. by advertised name;
  2. by advertised service (`AE30` cat, `18F0` ESC/POS, `AF30` Phomemo-or-cat, ISSC Clabel);
  3. after connecting, by the services the printer actually has, plus a paper-free question where needed: the Clabel version query, the MXW01 status query, or ESC/POS `DLE EOT` for unknown printers.

  It never sends the cat question to an ESC/POS printer (that prints "Qx"), and never writes down a Phomemo pipe before printing. The last printer used is preferred; the link is kept open; a link the printer dropped quietly is caught by acknowledging the first write of each job.
- **Bluetooth stack:** NimBLE-Arduino, not the ESP32 core's BLE library. The core library fails to connect to printers that start the MTU exchange themselves (the cat printer does), because it treats NimBLE's "already in progress" as an error and leaves the printer half-connected.
- **On/off detection:** the StickS3's power chip reports about 5 V on USB even on battery, so "plugged into a computer" is detected by the charging state instead. The camera never supplies power.

## Printer protocols

Raster input to every driver: 1 = black, leftmost dot in the high bit.

| Family | Pipe (service / write / notify) | Job | Pacing |
|---|---|---|---|
| Clabel CT-320B / CT-230B | ISSC `49535343-fe7d…` / `…8841…` / `…1e4d…` | TSPL label mode: `SIZE`, `GAP 0`, `DENSITY 8`, `SPEED 2`, `CLS`, `BITMAP x,y,bpr,rows,3,len,<raw deflate>` in bands of ≤40 rows, `PRINT 1,1`. 576 dots at 203 dpi / 568 at 300 dpi. | full-MTU writes, queue back-pressure; first and last write acknowledged |
| Marklife P50 | `FF00` / `FF02` / `FF01` | `1F C0 01 00`, `1F 10 wBE hBE lenBE` + **zlib with a 1 KB window** (header `28 15`; any other window gets `ER`), `1B 4A 64`, `1F C0 01 01`. Replies `AA 0D 0A` (taken) or `ER`. | 180 B, 30 ms gaps |
| Phomemo M110 family | `FF00` / `FF02` / `FF03` (must subscribe, or jobs are silently dropped) | `1B 4E 0D 03`, `1B 4E 04 08`, `1F 11 0B`, one `GS v 0` block (a split leaves a seam), footer `1F F0 05 00 1F F0 03 00` | 180 B, 5 ms gaps |
| Cat printers | `AE30` / `AE01` / `AE02` | `51 78 cmd 00 lenLE payload crc8 FF` frames; one `A2` frame per row with the **left dot in the low bit** | 128 B, 10 ms gaps |
| MXW01 | `AE30` / `AE01` + data `AE03` / `AE02` | `22 21 …` frames: intensity, print request (wait for its answer), rows on `AE03`, flush, wait for "printed" | 180 B, 15 ms gaps |
| BX1 | the **second** `FF00` service / `FF02` / `FF01` | `64 cmd seq lenLE payload 00 00 00 00 9B` frames, image 4 rows a frame | a steady 5,600 B/s |
| NETUM NT-861 | `18F0` / `2AF1` / `2AF0` | image **stored** (`GS ( L` fn 112), `GS I 1` asked and answered, then printed (fn 50), fed and cut. Image bytes that could begin a real-time command get one extra dot. | ≤25 KB/s (half if it echoes) |
| POS-8360 ("BlueTooth Printer") | ISSC | `ESC p` drawer pulses to hold it back, then `GS v 0`, blank feed, cut; `GS r 1` answered when done | ≤25 KB/s |
| Generic ESC/POS / unknown | `18F0`, ISSC or `FFE0` | `ESC @`, `GS v 0`, feed, at 384 dots | 180 B, 30 ms gaps for unknown printers |
