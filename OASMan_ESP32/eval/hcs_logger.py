#!/usr/bin/env python3
"""Serial logger for the OAS-Man manifold (Height Control Supervisor field data).

Runs anywhere with Python 3 and NO extra packages (Linux only: uses termios): a comma 3X / four on its AUX USB
port, a Raspberry Pi, a laptop. Timestamps every line with the wall clock, writes one file per day, reconnects
when the cable is unplugged or the manifold reboots, and deletes files older than --keep-days.

It opens the port without touching the DTR/RTS lines (pyserial / most terminals toggle them, which resets most
ESP32 boards). Opening a port may still reset some boards once -- try it while parked.

Docs: OASMan_ESP32/docs/hcs-field-data.md

usage: hcs_logger.py [--port /dev/ttyUSB0] [--dir ./hcs_logs] [--all] [--keep-days 60]
"""
import argparse
import datetime
import glob
import os
import select
import sys
import termios
import time

KEEP = ("HCS t=", "HCSD", "HRAW", "HCS supervisor", "HCS wheel")


def find_port():
    ports = sorted(glob.glob("/dev/ttyUSB*")) + sorted(glob.glob("/dev/ttyACM*"))
    return ports[0] if ports else None


def open_port(path, baud):
    fd = os.open(path, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    iflag, oflag, cflag, lflag, _, _, cc = attrs
    speed = getattr(termios, "B%d" % baud)
    iflag = 0
    oflag = 0
    lflag = 0
    cflag = (cflag & ~(termios.CSIZE | termios.PARENB | termios.CSTOPB | termios.HUPCL)) | termios.CS8 | termios.CREAD | termios.CLOCAL
    cc[termios.VMIN] = 0
    cc[termios.VTIME] = 0
    termios.tcsetattr(fd, termios.TCSANOW, [iflag, oflag, cflag, lflag, speed, speed, cc])
    return fd


def prune(directory, keep_days):
    cutoff = time.time() - keep_days * 86400
    for f in glob.glob(os.path.join(directory, "hcs-*.log")):
        try:
            if os.path.getmtime(f) < cutoff:
                os.remove(f)
        except OSError:
            pass


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="serial device (default: first /dev/ttyUSB* or /dev/ttyACM*)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--dir", default="hcs_logs", help="output directory (default ./hcs_logs)")
    ap.add_argument("--all", action="store_true", help="keep every line, not only the HCS ones")
    ap.add_argument("--keep-days", type=int, default=60)
    args = ap.parse_args()
    os.makedirs(args.dir, exist_ok=True)
    last_prune = 0.0
    while True:
        port = args.port or find_port()
        if not port or not os.path.exists(port):
            time.sleep(5)
            continue
        try:
            fd = open_port(port, args.baud)
        except OSError as e:
            print("open %s failed: %s" % (port, e), file=sys.stderr)
            time.sleep(5)
            continue
        print("logging %s -> %s" % (port, os.path.abspath(args.dir)), file=sys.stderr)
        buf = b""
        try:
            while True:
                if time.time() - last_prune > 3600:
                    prune(args.dir, args.keep_days)
                    last_prune = time.time()
                r, _, _ = select.select([fd], [], [], 5.0)
                if not r:
                    if not os.path.exists(port):
                        raise OSError("device gone")
                    continue
                chunk = os.read(fd, 4096)
                if not chunk:
                    raise OSError("device gone")  # readable but empty = hang-up
                buf += chunk
                if len(buf) > 65536:
                    buf = buf[-4096:]  # garbage without newlines (wrong baud?)
                while b"\n" in buf:
                    raw, buf = buf.split(b"\n", 1)
                    text = raw.decode("utf-8", "replace").rstrip("\r")
                    if not text or (not args.all and not any(k in text for k in KEEP)):
                        continue
                    now = datetime.datetime.now()
                    path = os.path.join(args.dir, now.strftime("hcs-%Y%m%d.log"))
                    with open(path, "a") as f:
                        f.write(now.strftime("%H:%M:%S.%f")[:-3] + " " + text + "\n")
        except OSError as e:
            print("port lost (%s), retrying" % e, file=sys.stderr)
            try:
                os.close(fd)
            except OSError:
                pass
            time.sleep(2)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
