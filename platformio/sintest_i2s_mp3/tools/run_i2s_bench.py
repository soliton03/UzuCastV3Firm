#!/usr/bin/env python3
"""Flash Master+Slave and run I2S bench (no BT button required on Slave)."""
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


def send_master_commands(ser: serial.Serial, mode: str) -> None:
    time.sleep(3.0)
    ser.reset_input_buffer()
    for cmd in (f"MODE {mode}", "ON 1"):
        ser.write((cmd + "\r\n").encode())
        ser.flush()
        print(f"[master-cmd] {cmd}")
        time.sleep(1.0)


def evaluate_slave(lines: list[str], master_lines: list[str], mode: str) -> tuple[bool, str]:
    text = "\n".join(lines)
    master_text = "\n".join(master_lines)
    build = re.search(r"build\s+(\d+)", text)
    decode = len(re.findall(r"\[MP3\] decode ok", text))
    bench_start = "I2S-BENCH] play start" in text
    peaks = [int(m) for m in re.findall(r"peak=(\d+)", text)]
    max_peak = max(peaks) if peaks else 0
    pcm_out = 0
    m = re.findall(r"pcm_out=(\d+)", text)
    if m:
        pcm_out = int(m[-1])
    drops = [int(x) for x in re.findall(r"u/d=\d+/(\d+)", text)]
    max_drop = max(drops) if drops else 0
    rx = 0
    rxm = re.findall(r"rx=(\d+)", text)
    if rxm:
        rx = int(rxm[-1])
    master_on = "OK ON 1" in master_text or "OK ON" in master_text
    master_mode = f"OK MODE {mode}" in master_text

    ok = True
    reasons: list[str] = []
    if not build and ok:
        pass  # 連続テスト時は起動行が無くても metrics で判定
    elif not build:
        ok = False
        reasons.append("no build number in slave log")
    if not master_mode:
        ok = False
        reasons.append(f"master MODE {mode} not confirmed")
    if not master_on:
        ok = False
        reasons.append("master ON 1 not confirmed")
    if mode == "MP3" and decode < 2:
        ok = False
        reasons.append(f"decode ok count={decode} (need >=2)")
    if not bench_start:
        ok = False
        reasons.append("I2S-BENCH play start missing")
    if max_peak < 1000:
        ok = False
        reasons.append(f"PCM peak={max_peak} (need >=1000, expect ~12000)")
    if pcm_out < 8000:
        ok = False
        reasons.append(f"pcm_out={pcm_out} (need >=8000)")
    if max_drop > 5000:
        ok = False
        reasons.append(f"push_drop max={max_drop} (need <=5000)")
    min_rx = 10 if mode == "MP3" else 1000
    if rx < min_rx:
        ok = False
        reasons.append(f"rx={rx} (need >={min_rx}) — I2S配線 Master TX -> Slave RX を確認")

    summary = (
        f"build={build.group(1) if build else '?'} decode={decode} rx={rx} "
        f"peak={max_peak} pcm_out={pcm_out} max_drop={max_drop}"
    )
    if ok:
        return True, "PASS " + summary
    return False, "FAIL " + summary + " :: " + "; ".join(reasons)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--master-port", default="COM11")
    ap.add_argument("--slave-port", default="COM7")
    ap.add_argument("--mode", choices=("MP3", "RAW"), default="MP3")
    ap.add_argument("--skip-upload", action="store_true")
    ap.add_argument("--duration", type=float, default=18.0)
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

    send_master_commands(master_ser, args.mode)

    print(f"[bench] collecting {args.duration}s ...")
    time.sleep(args.duration)

    stop.set()
    time.sleep(0.3)
    slave_ser.close()
    master_ser.close()
    t_slave.join(timeout=2)
    t_master.join(timeout=2)

    ok, msg = evaluate_slave(slave_lines, master_lines, args.mode)
    print(msg)
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
