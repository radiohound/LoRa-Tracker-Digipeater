#!/usr/bin/env python3
"""Flash a firmware .bin into an STM32WLE5 through its built-in serial bootloader (ST AN3155), on Linux/macOS.

  1. Hold the board's bootloader key (Key 1 on the MBL baseboard, PA1), tap Reset, keep holding a second.
  2. python3 tools/flash_serial.py /dev/ttyUSB0 lora-aprs-digi-e77.bin

Only the pages the image needs are erased, so the settings in the last 2 KB page are kept.
Needs no extra modules (Python 3 standard library only)."""
import os, select, sys, termios, time

FLASH = 0x08000000
PAGE = 2048
ACK, NACK = 0x79, 0x1F


def open_port(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    a = termios.tcgetattr(fd)
    a[0] = a[1] = a[3] = 0
    a[2] = termios.CS8 | termios.CREAD | termios.CLOCAL | termios.PARENB     # 8E1: what the bootloader uses
    a[4] = a[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, a)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def rx(fd, n, timeout=2.0):
    out = b""
    end = time.time() + timeout
    while len(out) < n and time.time() < end:
        if select.select([fd], [], [], 0.05)[0]:
            try:
                out += os.read(fd, n - len(out))
            except BlockingIOError:
                pass
    if len(out) < n:
        raise SystemExit("timeout: got %d of %d bytes (is the board in the bootloader?)" % (len(out), n))
    return out


def ack(fd, timeout=2.0, what=""):
    b = rx(fd, 1, timeout)[0]
    if b != ACK:
        raise SystemExit("no ACK%s (got 0x%02X)" % (" for " + what if what else "", b))


def cmd(fd, c):
    os.write(fd, bytes([c, c ^ 0xFF]))
    ack(fd, 2.0, "command 0x%02X" % c)


def xor(bs):
    x = 0
    for b in bs:
        x ^= b
    return x


def addr_bytes(a):
    b = a.to_bytes(4, "big")
    return b + bytes([xor(b)])


def main():
    if len(sys.argv) != 3:
        raise SystemExit(__doc__)
    port, binpath = sys.argv[1], sys.argv[2]
    data = open(binpath, "rb").read()
    if len(data) % 4:
        data += b"\xff" * (4 - len(data) % 4)
    pages = (len(data) + PAGE - 1) // PAGE
    fd = open_port(port)
    os.write(fd, b"\x7f")                                 # the start byte; a bootloader already woken up does not answer it twice
    if select.select([fd], [], [], 0.7)[0]:
        os.read(fd, 8)
    cmd(fd, 0x00)                                        # Get: version and the commands it supports
    n = rx(fd, 1)[0]
    info = rx(fd, n + 1)
    ack(fd)
    print("bootloader version 0x%02X, commands %s" % (info[0], " ".join("%02X" % c for c in info[1:])))
    cmd(fd, 0x44)                                         # Extended Erase, a list of pages
    body = (pages - 1).to_bytes(2, "big") + b"".join(p.to_bytes(2, "big") for p in range(pages))
    os.write(fd, body + bytes([xor(body)]))
    ack(fd, 30.0, "erase")
    print("erased %d pages (%d KB)" % (pages, pages * PAGE // 1024))
    # The first page goes in LAST. Until then the start of flash is blank, and a blank chip starts its own
    # bootloader: an update that is interrupted halfway can simply be run again, with no ST-LINK.
    first = min(PAGE, len(data))
    order = list(range(first, len(data), 256)) + list(range(0, first, 256))

    def write_block(off):
        chunk = data[off:off + 256]
        cmd(fd, 0x31)
        os.write(fd, addr_bytes(FLASH + off)); ack(fd, 2.0, "address")
        pl = bytes([len(chunk) - 1]) + chunk
        os.write(fd, pl + bytes([xor(pl)])); ack(fd, 5.0, "write at 0x%08X" % (FLASH + off))

    def check_block(off):
        chunk = data[off:off + 256]
        cmd(fd, 0x11)
        os.write(fd, addr_bytes(FLASH + off)); ack(fd, 2.0, "read address")
        os.write(fd, bytes([len(chunk) - 1, (len(chunk) - 1) ^ 0xFF])); ack(fd, 2.0, "read length")
        return rx(fd, len(chunk)) == chunk

    rest = order[:len(order) - len(range(0, first, 256))]
    head = order[len(rest):]
    for n, off in enumerate(rest):
        write_block(off)
        if n % 32 == 0:
            print("\rwriting %3d%%" % (100 * n // max(1, len(order))), end="", flush=True)
    bad = sum(0 if check_block(off) else 1 for off in rest)
    if bad:
        raise SystemExit("\nverify FAILED in %d blocks: the first page was NOT written, so the chip stays blank" % bad)
    for off in head:
        write_block(off)
    bad = sum(0 if check_block(off) else 1 for off in head)
    print("\rwritten      ")
    if bad:
        raise SystemExit("verify FAILED in the first page")
    print("verified OK")
    cmd(fd, 0x21)                                         # Go: start the program
    os.write(fd, addr_bytes(FLASH)); ack(fd, 2.0, "go")
    print("started the program")


main()
