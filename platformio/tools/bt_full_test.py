import serial
import time
import threading
import sys

slave_log = []
stop = False


def read_slave(ser):
    global stop
    while not stop:
        try:
            line = ser.readline()
            if line:
                text = line.decode("utf-8", errors="replace").rstrip()
                slave_log.append(text)
                print(f"[COM4] {text}")
        except Exception as e:
            if not stop:
                print(f"read err: {e}")
            break


def drain_main(ser, seconds=1.0):
    end = time.time() + seconds
    while time.time() < end:
        while ser.in_waiting:
            print(f"[COM5] {ser.readline().decode('utf-8', errors='replace').rstrip()}")
        time.sleep(0.05)


def main():
    global stop
    try:
        sub = serial.Serial("COM4", 115200, timeout=0.5)
        m = serial.Serial("COM5", 115200, timeout=0.5)
    except Exception as e:
        print(f"open failed: {e}")
        sys.exit(1)

    time.sleep(2)
    t = threading.Thread(target=read_slave, args=(sub,), daemon=True)
    t.start()

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

    print("[TEST] monitoring 45s...")
    end = time.time() + 45
    while time.time() < end:
        time.sleep(0.2)

    stop = True
    time.sleep(0.5)
    sub.close()
    m.close()

    print("--- SUMMARY ---")
    for line in slave_log:
        if any(k in line for k in ["[PCM]", "I2S tag hb", "MEDIA", "audio_state", "CONNECT", "BENCH", "drop=", "build"]):
            print(line)


if __name__ == "__main__":
    main()
