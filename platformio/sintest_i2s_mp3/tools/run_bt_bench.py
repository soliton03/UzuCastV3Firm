#!/usr/bin/env python3
"""Flash Master+Slave, auto BT connect via serial BT GO, verify MP3 over A2DP path."""
from __future__ import annotations

import argparse
import re
import subprocess
import sys
import threading
import time
from pathlib import Path

try:
    import serial
except ImportError:
    print("pip install pyserial", file=sys.stderr)
    sys.exit(1)

ROOT = Path(__file__).resolve().parents[1]
PIO = Path.home() / ".platformio" / "penv" / "Scripts" / "python.exe"
MASTER_DIR = ROOT / "master"
SLAVE_DIR = ROOT / "slave"


def pio_upload(project: Path, port: str) -> None:
    env = {
        **dict(subprocess.os.environ),
        "PYTHONIOENCODING": "utf-8",
        "PYTHONUTF8": "1",
        "PLATFORMIO_FORCE_ANSI": "0",
    }
    cmd = [str(PIO), "-m", "platformio", "run", "-d", str(project), "-t", "upload", "--upload-port", port]
    print(f"[upload] {' '.join(cmd)}")
    subprocess.run(cmd, check=True, env=env)


def read_lines(ser: serial.Serial, bucket: list[str], label: str, stop: threading.Event) -> None:
    while not stop.is_set():
        try:
            if not ser.is_open:
                break
            raw = ser.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").rstrip()
        except (serial.SerialException, TypeError, OSError):
            break
        if line:
            bucket.append(line)
            print(f"[{label}] {line}")


def send_line(ser: serial.Serial, cmd: str, label: str) -> None:
    ser.write((cmd + "\r\n").encode())
    ser.flush()
    print(f"[{label}-cmd] {cmd}")
    time.sleep(0.5)


def evaluate(slave_lines: list[str], master_lines: list[str]) -> tuple[bool, str]:
    text = "\n".join(slave_lines)
    master_text = "\n".join(master_lines)

    build = re.search(r"build\s+(\d+)", text)
    decode = len(re.findall(r"\[MP3\] decode ok", text))
    bt_go = "OK BT GO" in text
    connected = "STATE PAIRING -> CONNECT" in text or "STATE CONNECTING -> CONNECT" in text
    media = "MEDIA START" in text
    bench_start = "I2S-BENCH] play start" in text
    peaks = [int(m) for m in re.findall(r"peak=(\d+)", text)]
    max_peak = max(peaks) if peaks else 0
    drops = [int(x) for x in re.findall(r"u/d=\d+/(\d+)", text)]
    max_drop = max(drops) if drops else 0
    play_on = any("play=1" in ln for ln in slave_lines if "I2S tag hb" in ln)
    master_ok = "OK MODE MP3" in master_text and "OK ON 1" in master_text

    ok = True
    reasons: list[str] = []
    if not build:
        ok = False
        reasons.append("no build number")
    elif build and int(build.group(1)) < 88:
        ok = False
        reasons.append(f"build={build.group(1)} (need >=88)")
    if not bt_go:
        ok = False
        reasons.append("BT GO not acknowledged")
    if not master_ok:
        ok = False
        reasons.append("master MODE MP3 / ON 1 failed")
    if not connected:
        ok = False
        reasons.append("BT CONNECT not reached")
    if bench_start:
        ok = False
        reasons.append("I2S-BENCH started (expected real BT pcmPlaybackBegin path)")
    if not media:
        ok = False
        reasons.append("MEDIA START missing")
    if decode < 2:
        ok = False
        reasons.append(f"decode ok={decode} (need >=2)")
    if not play_on:
        ok = False
        reasons.append("play=1 missing in heartbeat")
    if max_peak < 1000:
        ok = False
        reasons.append(f"peak={max_peak} (need >=1000)")
    if max_drop > 5000:
        ok = False
        reasons.append(f"push_drop={max_drop} (need <=5000)")

    summary = (
        f"build={build.group(1) if build else '?'} decode={decode} "
        f"peak={max_peak} max_drop={max_drop} media={int(media)} play={int(play_on)}"
    )
    if ok:
        return True, "PASS " + summary
    return False, "FAIL " + summary + " :: " + "; ".join(reasons)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--master-port", default="COM11")
    ap.add_argument("--slave-port", default="COM7")
    ap.add_argument("--skip-upload", action="store_true")
    ap.add_argument("--duration", type=float, default=55.0)
    args = ap.parse_args()

    if not args.skip_upload:
        pio_upload(SLAVE_DIR, args.slave_port)
        pio_upload(MASTER_DIR, args.master_port)
        time.sleep(2.0)

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

    time.sleep(4.0)
    send_line(slave_ser, "BT GO", "slave")
    time.sleep(2.0)
    send_line(master_ser, "MODE MP3", "master")
    send_line(master_ser, "ON 1", "master")

    print(f"[bt-bench] collecting {args.duration}s ...")
    time.sleep(args.duration)

    stop.set()
    time.sleep(0.3)
    slave_ser.close()
    master_ser.close()
    t_slave.join(timeout=2)
    t_master.join(timeout=2)

    ok, msg = evaluate(slave_lines, master_lines)
    print(msg)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
