import re
import serial
import sys
import threading
import time

MONITOR_SEC = 180
SLAVE_PORT = "COM4"
MAIN_PORT = "COM5"

slave_log = []
main_log = []
stop = False

PCM_RE = re.compile(
    r"hold=(\d+) hmin=(\d+) cbout=(\d+) underrun=(\d+) drop=(\d+)"
)
HB_RE = re.compile(
    r"rx=(\d+) ok=(\d+) err=(\d+) .* hold=(\d+) hmin=(\d+) u/d=(\d+)/(\d+)"
)


def read_port(ser, bucket, tag):
    global stop
    while not stop:
        try:
            line = ser.readline()
            if not line:
                continue
            text = line.decode("utf-8", errors="replace").rstrip()
            bucket.append(text)
            if any(
                k in text
                for k in (
                    "[PCM]",
                    "I2S tag hb",
                    "MEDIA",
                    "Guru",
                    "panic",
                    "TDM write",
                    "write failed",
                    "Rebooting",
                )
            ):
                print(f"[{tag}] {text}")
        except Exception as e:
            if not stop:
                print(f"[{tag}] read err: {e}")
            break


def drain_main(ser, seconds=1.0):
    end = time.time() + seconds
    while time.time() < end:
        while ser.in_waiting:
            text = ser.readline().decode("utf-8", errors="replace").rstrip()
            main_log.append(text)
            print(f"[COM5] {text}")
        time.sleep(0.05)


def analyze():
    pcm_samples = []
    hb_samples = []
    events = []

    for line in slave_log:
        if "Guru" in line or "panic" in line or "Rebooting" in line:
            events.append(line)
        m = PCM_RE.search(line)
        if m:
            pcm_samples.append(tuple(int(x) for x in m.groups()))
        m = HB_RE.search(line)
        if m:
            hb_samples.append(tuple(int(x) for x in m.groups()))

    for line in main_log:
        if "TDM write failed" in line or "write failed" in line:
            events.append(f"[MAIN] {line}")

    print("\n=== ANALYSIS (%ds) ===" % MONITOR_SEC)
    print("slave lines=%d main lines=%d pcm_samples=%d hb_samples=%d" % (
        len(slave_log), len(main_log), len(pcm_samples), len(hb_samples)
    ))

    if pcm_samples:
        holds = [s[0] for s in pcm_samples]
        hmins = [s[1] for s in pcm_samples]
        cbouts = [s[2] for s in pcm_samples]
        underruns = [s[3] for s in pcm_samples]
        drops = [s[4] for s in pcm_samples]
        print(
            "PCM hold: min=%d max=%d last=%d" % (min(holds), max(holds), holds[-1])
        )
        print(
            "PCM hmin: min=%d max=%d last=%d" % (min(hmins), max(hmins), hmins[-1])
        )
        print(
            "PCM cbout: start=%d end=%d delta=%d"
            % (cbouts[0], cbouts[-1], cbouts[-1] - cbouts[0])
        )
        print(
            "PCM underrun: start=%d end=%d delta=%d"
            % (underruns[0], underruns[-1], underruns[-1] - underruns[0])
        )
        print(
            "PCM drop: start=%d end=%d delta=%d"
            % (drops[0], drops[-1], drops[-1] - drops[0])
        )
        low_hmin = [h for h in hmins if h < 512]
        if low_hmin:
            print("WARNING: hmin dipped below 512 (%d times, min=%d)" % (
                len(low_hmin), min(low_hmin)
            ))

    if hb_samples:
        rx = [s[0] for s in hb_samples]
        errs = [s[2] for s in hb_samples]
        print("I2S rx: start=%d end=%d delta=%d err_total=%d" % (
            rx[0], rx[-1], rx[-1] - rx[0], sum(errs)
        ))

    if events:
        print("EVENTS:")
        for e in events:
            print("  " + e)
    else:
        print("EVENTS: none")


def main():
    global stop
    try:
        sub = serial.Serial(SLAVE_PORT, 115200, timeout=0.5)
        m = serial.Serial(MAIN_PORT, 115200, timeout=0.5)
    except Exception as e:
        print("open failed: %s" % e)
        sys.exit(1)

    time.sleep(2)
    threading.Thread(target=read_port, args=(sub, slave_log, "COM4"), daemon=True).start()
    threading.Thread(target=read_port, args=(m, main_log, "COM5"), daemon=True).start()

    drain_main(m, 2)

    print("[TEST] PIPE RST on COM4")
    sub.write(b"PIPE RST\r\n")
    time.sleep(1)

    print("[TEST] BT GO on COM4")
    sub.write(b"BT GO\r\n")
    time.sleep(10)

    print("[TEST] dir on COM5")
    m.write(b"dir\r\n")
    drain_main(m, 2)

    print("[TEST] play 2 on COM5")
    m.write(b"play 2\r\n")
    drain_main(m, 1)

    print("[TEST] monitoring %ds..." % MONITOR_SEC)
    end = time.time() + MONITOR_SEC
    while time.time() < end:
        time.sleep(0.5)

    stop = True
    time.sleep(0.5)
    sub.close()
    m.close()
    analyze()


if __name__ == "__main__":
    main()
