#!/usr/bin/env python3
"""Test MP3 path when BT is already connected (no BT GO / no upload)."""
from __future__ import annotations

import argparse
import re
import sys
import threading
import time

try:
    import serial
except ImportError:
    print("pip install pyserial", file=sys.stderr)
    sys.exit(1)


def read_lines(ser: serial.Serial, bucket: list[str], label: str, stop: threading.Event) -> None:
    while not stop.is_set():
        try:
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").rstrip()
        except (serial.SerialException, TypeError, OSError):
            break
        if line:
            bucket.append(line)
            print(f"[{label}] {line}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--master-port", default="COM11")
    ap.add_argument("--slave-port", default="COM7")
    ap.add_argument("--duration", type=float, default=35.0)
    args = ap.parse_args()

    slave_lines: list[str] = []
    master_lines: list[str] = []
    stop = threading.Event()

    slave_ser = serial.Serial(args.slave_port, 115200, timeout=0.2)
    master_ser = serial.Serial(args.master_port, 115200, timeout=0.2)

    t_slave = threading.Thread(
        target=read_lines, args=(slave_ser, slave_lines, "slave", stop), daemon=True
    )
    t_master = threading.Thread(
        target=read_lines, args=(master_ser, master_lines, "master", stop), daemon=True
    )
    t_slave.start()
    t_master.start()

    time.sleep(1.0)
    for cmd in ("MODE MP3", "ON 1"):
        master_ser.write((cmd + "\r\n").encode())
        master_ser.flush()
        print(f"[master-cmd] {cmd}")
        time.sleep(1.0)

    print(f"[live-bt] collecting {args.duration}s (BT assumed connected)...")
    time.sleep(args.duration)

    stop.set()
    time.sleep(0.3)
    slave_ser.close()
    master_ser.close()
    t_slave.join(timeout=2)
    t_master.join(timeout=2)

    text = "\n".join(slave_lines)
    master_text = "\n".join(master_lines)
    build = re.search(r"build\s+(\d+)", text)
    decode = len(re.findall(r"\[MP3\] decode ok", text))
    media = "MEDIA START" in text
    bench = "I2S-BENCH] play start" in text
    peaks = [int(m) for m in re.findall(r"peak=(\d+)", text)]
    max_peak = max(peaks) if peaks else 0
    drops = [int(x) for x in re.findall(r"u/d=\d+/(\d+)", text)]
    max_drop = max(drops) if drops else 0
    play_on = any("play=1" in ln for ln in slave_lines if "I2S tag hb" in ln)
    hb = [ln for ln in slave_lines if "I2S tag hb" in ln][-3:]

    print("--- last heartbeats ---")
    for ln in hb:
        print(ln)

    ok = True
    reasons: list[str] = []
    if "OK ON 1" not in master_text:
        ok = False
        reasons.append("master ON 1 not confirmed")
    if decode < 1:
        ok = False
        reasons.append(f"decode={decode}")
    if bench:
        ok = False
        reasons.append("I2S-BENCH active (BT path not used?)")
    if not play_on:
        ok = False
        reasons.append("play=1 missing")
    if max_peak < 1000:
        ok = False
        reasons.append(f"peak={max_peak}")
    if max_drop > 5000:
        ok = False
        reasons.append(f"push_drop={max_drop}")

    summary = (
        f"build={build.group(1) if build else '?'} media={int(media)} "
        f"decode={decode} peak={max_peak} max_drop={max_drop} play={int(play_on)}"
    )
    msg = ("PASS " if ok else "FAIL ") + summary
    if reasons:
        msg += " :: " + "; ".join(reasons)
    print(msg)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
