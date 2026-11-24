@ "Shot sync" patch. VA = file offset - 0x200.
@ Assembles for the v006 layout (SP140A-sensor cameras) by default, or for the
@ v012 layout (OV9732-sensor cameras, e.g. the blue one) with --defsym V012=1.
@
@ The camera stays a normal camera. Right after each photo is saved it
@ pretends a USB cable was plugged in, so the stock firmware probes for a host
@ and, if one answers, presents the SD card as a USB disk. When the window ends
@ the camera restarts itself straight into photo mode.
@
@ Why restart: stock firmware never returns from USB-disk mode (it powers off
@ after USB). USB entry stops the sensor clock, changes system clock bits and
@ more; restoring the known pieces in place was not enough. Boot is the one
@ path known to bring everything up, so the camera re-runs the
@ firmware's own reset handler (a hardware/watchdog reset drops the power hold
@ and the camera switches off).
@
@ Hook sites:
@   hook   : the `bl` in the photo state's (0x102) 0x500e handler that finishes a
@            capture and closes the file. Arms the window, then continues.
@   cave   : the DC-in read (`bl 0x6d5c` at 0x7a20) used by the USB plug state
@            machine. Returns real DC-in, or 1 while the window is open.
@   tick   : `bl 0x7d50` at 0x87a4 in the peripheral task loop (runs on every
@            pass, unlike the plug state machine, which stops in USB mode).
@            Ends the window: after a USB session it sets the restart marker and
@            restarts the firmware in software.
@   bootdc : `bl 0x6d5c` at 0x60e4 in the boot power check (0x60a8), right after
@            RTC register 0x83 (stock: low-battery marker 0xA5) is read into the
@            caller's [sp+4]. If it holds RESTART_MARK, clear it, remember the
@            request and answer "DC present" so boot does not wait for the
@            power key (no one is holding it) and switch off.
@   nextst : `ldr r1, =0x10d` at 0x6384 at the end of startup (0x6378, one
@            caller). On a requested restart, go to photo mode (0x102) instead
@            of the mode screen (0x10d).
@
@ Things learned the hard way:
@   - State 0x101 (0x1c444) is video; photo mode is state 0x102 (0x1d4fc).
@   - State 0x108 (0x21170) is the SD firmware updater, not USB.
@   - 0xf573c, used by earlier patches, is the flat top of a lookup table, not
@     free space. This code lives inside lwIP's dns_recv, which is dead because
@     the Wi-Fi stack is never started; its variables live in lwIP's dns_table
@     (zeroed RAM the camera never uses).
    .syntax unified
    .arm
    .ifdef V012
    .equ BOOT_DC_SITE, 0x60e4
    .equ NEXT_STATE,   0x6384
    .equ CAPTURE_DONE, 0x1d294
    .equ HOOK_SITE,    0x1d824
    .equ msgQSend,     0x25990     @ (queue, msg, ptr, len, flag)
    .equ RTC_WRITE,    0x2e9b0     @ (reg, byte)
    .equ SPI_READ,     0x306c0     @ (0, flash addr, dst, len): page reads, works with IRQs off
    .equ USB_IRQ_OFF,  0x8a328     @ masks the USB interrupt (0x35)
    .equ POWER_OFF,    0x30d0c     @ stock power-off: release power hold, system reset
    .equ DATA_FLASH,   0xffb00     @ flash pages covering .data (VA 0xff960 + 0xb390)
    .equ DATA_VA,      0xff900
    .equ DATA_LEN,     0xb400
    .equ BOOT_SP,      0x1b0800    @ inside the reset handler's SVC stack (0x1ae87f..0x1b0882)
    .equ CAVE,         0x4b520     @ inside dns_recv (0x4b51c)
    .equ OSTimeGet,    0x9076c     @ 100 Hz ticks
    .equ PLUG_VARS,    0x0010ca18  @ +0xc plug state, +0xd "in USB mode" flag
    .equ VARS,         0x00120154
    .else
    .equ BOOT_DC_SITE, 0x60e4
    .equ NEXT_STATE,   0x6384
    .equ CAPTURE_DONE, 0x1d27c
    .equ HOOK_SITE,    0x1d80c
    .equ msgQSend,     0x25984
    .equ RTC_WRITE,    0x2e9a4
    .equ SPI_READ,     0x306b4
    .equ USB_IRQ_OFF,  0x8a274
    .equ POWER_OFF,    0x30d00
    .equ DATA_FLASH,   0xffa00     @ .data VA 0xff870 + 0xb6c0
    .equ DATA_VA,      0xff800
    .equ DATA_LEN,     0xb800
    .equ BOOT_SP,      0x1b0a40    @ SVC stack 0x1aeabf..0x1b0ac2
    .equ CAVE,         0x4b514     @ inside dns_recv (0x4b510)
    .equ OSTimeGet,    0x906b8
    .equ PLUG_VARS,    0x0010cc58
    .equ VARS,         0x00120394
    .endif
    .equ DC_READ,      0x6d5c
    .equ RESET_ENTRY,  0x3c        @ the firmware's reset handler
    .equ ARM_TICKS,    5           @ photo saved -> window opens (50 ms)
    .equ OPEN_TICKS,   400         @ window length (4 s); the bridge needs well under 1 s
    .equ RESTART_MARK, 0x5a
    .equ PLUG_DEBOUNCE, 17         @ plug state machine accepts a change after 16 runs

    .org BOOT_DC_SITE
    bl   bootdc
    .org NEXT_STATE
    bl   nextst
    .org DC_READ
dc_read:
    .org 0x7a20
    bl   cave
    .org 0x7d50
periph_next:
    .org 0x87a4
    bl   tick
    .org CAPTURE_DONE
capture_done:
    .org HOOK_SITE
    bl   hook
    .org msgQSend
msg_q_send:
    .org RTC_WRITE
rtc_write:
    .org SPI_READ
spi_read:
    .org POWER_OFF
power_off:

    .org CAVE
@ VARS: [0] phase (0 idle, 1 armed, 2 window open), [4] deadline in OS ticks,
@       [8] 0xD0000008 and [16] 0xD000000C as they were when the photo was saved,
@       [12] restart requested
hook:
    push {r0, r1, r2, lr}
    bl   os_time_get
    add  r0, r0, #ARM_TICKS
    ldr  r1, =VARS
    str  r0, [r1, #4]
    mov  r0, #1
    str  r0, [r1]
    mov  r2, #0xd0000000
    ldr  r0, [r2, #0x08]
    str  r0, [r1, #8]
    ldr  r0, [r2, #0x0c]
    str  r0, [r1, #16]
    pop  {r0, r1, r2, lr}
    b    capture_done

cave:
    push {r4, r5, lr}
    mov  r1, #0xd0000000
    ldr  r5, [r1, #0x68]
    mov  r5, r5, lsr #2
    and  r5, r5, #1                @ r5 = real DC-in
    ldr  r4, =VARS
    ldr  r1, [r4]
    cmp  r1, #2
    moveq r5, #1                   @ window open
    cmp  r1, #1
    bne  1f
    bl   os_time_get               @ armed: open once the deadline has passed
    ldr  r2, [r4, #4]
    subs r3, r0, r2
    bmi  1f
    mov  r1, #2
    str  r1, [r4]
    add  r0, r0, #OPEN_TICKS
    str  r0, [r4, #4]
    mov  r5, #1
    ldr  r1, =PLUG_VARS            @ skip the plug state machine's ~0.5 s debounce
    mov  r2, #PLUG_DEBOUNCE
    str  r2, [r1, #0x58]
1:  mov  r0, r5
    pop  {r4, r5, pc}

tick:
    push {r4, lr}
    ldr  r4, =VARS
    ldr  r1, [r4]
    cmp  r1, #2
    bne  2f
    bl   os_time_get
    ldr  r2, [r4, #4]
    subs r3, r0, r2
    bmi  2f
    mov  r2, #0                    @ window over
    str  r2, [r4]
    mov  r1, #0xd0000000
    ldr  r1, [r1, #0x68]
    tst  r1, #4
    bne  2f                        @ real power on the cable: stock logic owns it
    ldr  r0, [r4, #8]              @ the USB probe (0x8a2d0) clears bits 8 and 15 of
    and  r0, r0, #0x8100           @ 0xD0000008 and nothing sets them again; put
    mov  r1, #0xd0000000           @ them back (matters when no host answered and
    ldr  r3, [r1, #0x08]           @ the camera stays up)
    bic  r3, r3, #0x8100
    orr  r3, r3, r0
    str  r3, [r1, #0x08]
    ldr  r4, =PLUG_VARS
    strb r2, [r4, #12]             @ plug state machine back to idle
    str  r2, [r4, #0x64]
    str  r2, [r4, #0x58]
    str  r2, [r4, #0x5c]
    ldrb r1, [r4, #13]             @ did the camera actually enter USB mode?
    cmp  r1, #0
    beq  2f
    mov  r0, #0x83                 @ yes: ask the next boot to go straight to photo mode
    mov  r1, #RESTART_MARK
    bl   rtc_write
    .ifndef SOFT_RESTART
    @ Power off with the stock routine. A software restart (--defsym
    @ SOFT_RESTART=1) comes back with a dead image pipeline: USB mode changes
    @ hardware state that only a real power-on resets. With the marker set,
    @ one press of the power key boots straight into the viewfinder.
    bl   power_off
    .endif
    bl   usb_irq_off
    ldr  r4, =VARS                 @ USB connect (0x7b590) stopped the sensor clock:
    ldr  r0, [r4, #16]             @ put bits 8, 11, 14 of 0xD000000C back for the boot
    and  r0, r0, #0x4900
    mov  r1, #0xd0000000
    ldr  r3, [r1, #0x0c]
    bic  r3, r3, #0x4900
    orr  r3, r3, r0
    str  r3, [r1, #0x0c]
    @ Software restart. A hardware reset (watchdog) drops the power hold and the
    @ camera switches off, so re-run the firmware's own reset handler instead.
    @ It zeroes BSS but does not re-copy .data (it runs in place), so reload the
    @ flash pages that hold .data's initial values first. Done on the boot stack
    @ area, which is idle while tasks run, because the reload and the BSS clear
    @ may cover the current task's stack.
    mrs  r0, cpsr
    orr  r0, r0, #0xc0
    msr  cpsr_c, r0
    ldr  sp, =BOOT_SP
    mov  r0, #0
    ldr  r1, =DATA_FLASH
    ldr  r2, =DATA_VA
    ldr  r3, =DATA_LEN
    bl   spi_read
    ldr  pc, =RESET_ENTRY
2:  pop  {r4, lr}
    b    periph_next

@ Boot power check: the caller has just read RTC 0x83 into its [sp+4]; our sp
@ is the caller's sp because `bl` does not touch it.
bootdc:
    ldrb r1, [sp, #4]
    cmp  r1, #RESTART_MARK
    bne  dc_read
    push {r4, lr}
    mov  r0, #0x83
    mov  r1, #0
    bl   rtc_write                 @ one-shot
    ldr  r4, =VARS
    mov  r0, #1
    str  r0, [r4, #12]
    pop  {r4, pc}                  @ r0 = 1: "DC present", boot continues

@ End of startup: replaces `ldr r1, =0x10d`; r0 holds the dispatcher queue.
nextst:
    ldr  r1, =VARS
    ldr  r2, [r1, #12]
    cmp  r2, #0
    beq  4f
    mov  r2, #0
    str  r2, [r1, #12]
    ldr  r1, =0x102                @ photo mode
    mov  pc, lr
4:  ldr  r1, =0x10d                @ mode screen
    mov  pc, lr
    .ltorg
    .word 0xCAFEF00D               @ end marker for tools/build_novbus.py

    .org USB_IRQ_OFF
usb_irq_off:
    .org OSTimeGet
os_time_get:
