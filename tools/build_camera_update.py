#!/usr/bin/env python3
"""Build a Charmera SD-card update with the shot-sync patch (patch/shotsync.s).

usage: build_camera_update.py {v012|v006} <base image> [--out DIR]

  v012  cameras with the OV9732 sensor. Base: charmera-v012-2026-01-12-clean.bin
        from https://github.com/the-dumping-grounds/charmera-firmware-v012
  v006  cameras with the SP140A sensor (EXPERIMENTAL, never run on a camera).
        Base: a v006-family stock dump, e.g. the public one in
        https://github.com/benbadio/keychain-camera-plus (dumps/)

Needs the GNU Arm assembler on PATH (arm-none-eabi-as, arm-none-eabi-objcopy).

The update file is the first 0x1E0000 bytes of flash with the patch applied,
named JH_5408<stamp>_<SUM8>.bin, where SUM8 is the 32-bit byte sum of the file
(the camera's updater checks it) and the stamp must differ from the firmware's
own 202505201029 (otherwise the updater ignores the file). Every hook site is
checked against the bytes the patch expects to replace, so a base image of the
wrong family or with unexpected changes is refused instead of patched.
"""
import argparse, datetime, os, shutil, subprocess, sys, tempfile

UPD_LEN = 0x1E0000
HDR = 0x200                     # file offset = VA + 0x200
ROOT = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
PATCH = os.path.join(ROOT, 'patch', 'shotsync.s')

# Hook sites (VA -> original bytes) and the code cave (inside lwIP's dead
# dns_recv) for each firmware family.
VARIANTS = {
    'v012': dict(defsyms=['V012=1'], cave=(0x4b520, 0x4b8f0),
                 hooks={0x60e4: '1c0300eb', 0x6384: '88119fe5', 0x7a20: 'cdfcffeb', 0x87a4: '69fdffeb', 0x1d824: '9afeffeb'}),
    'v006': dict(defsyms=[], cave=(0x4b514, 0x4b8e4),
                 hooks={0x60e4: '1c0300eb', 0x6384: '88119fe5', 0x7a20: 'cdfcffeb', 0x87a4: '69fdffeb', 0x1d80c: '9afeffeb'}),
}
END_MARK = bytes.fromhex('0df0feca')   # .word 0xCAFEF00D at the end of the patch code


def assemble(defsyms):
    for tool in ('arm-none-eabi-as', 'arm-none-eabi-objcopy'):
        if not shutil.which(tool):
            sys.exit(f'{tool} not found: install the GNU Arm toolchain (e.g. brew install --cask gcc-arm-embedded)')
    with tempfile.TemporaryDirectory() as tmp:
        obj, binf = os.path.join(tmp, 'p.o'), os.path.join(tmp, 'p.bin')
        cmd = ['arm-none-eabi-as', '-mcpu=arm7tdmi']
        for d in defsyms:
            cmd += ['--defsym', d]
        subprocess.run(cmd + ['-o', obj, PATCH], check=True)
        subprocess.run(['arm-none-eabi-objcopy', '-O', 'binary', obj, binf], check=True)
        return open(binf, 'rb').read()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('variant', choices=VARIANTS)
    ap.add_argument('base')
    ap.add_argument('--out', default=os.path.join(ROOT, 'build'))
    a = ap.parse_args()
    v = VARIANTS[a.variant]

    d = bytearray(open(a.base, 'rb').read()[:UPD_LEN])
    if len(d) != UPD_LEN:
        sys.exit(f'base image is shorter than 0x{UPD_LEN:X} bytes')
    patch = assemble(v['defsyms'])

    for va, orig in v['hooks'].items():
        cur = bytes(d[va + HDR:va + HDR + 4])
        if cur != bytes.fromhex(orig):
            sys.exit(f'refusing: at VA 0x{va:x} expected {orig}, found {cur.hex()}. '
                     f'Is this a {a.variant} base image, and unmodified?')
        d[va + HDR:va + HDR + 4] = patch[va:va + 4]
    lo, hi = v['cave']
    end = patch.find(END_MARK, lo)
    if not lo < end < hi:
        sys.exit('patch code does not fit its cave')
    d[lo + HDR:end + 4 + HDR] = patch[lo:end + 4]

    s = sum(d) & 0xFFFFFFFF
    stamp = datetime.datetime.now().strftime('%Y%m%d%H%M')
    os.makedirs(a.out, exist_ok=True)
    out = os.path.join(a.out, f'JH_5408{stamp}_{s:08X}.bin')
    open(out, 'wb').write(d)
    print(f'{a.variant} shot-sync update: {out}')
    print('Copy it to the root of the camera\'s microSD card (nothing else named JH_*), switch the camera on,')
    print('and wait for "Upgrade 100%". Then remove the card and DELETE the file, or it flashes again at every boot.')


if __name__ == '__main__':
    main()
