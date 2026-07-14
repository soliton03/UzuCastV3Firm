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

    for _ in range(30):
        if sub.in_waiting:
            line = sub.readline().decode("utf-8", errors="replace").rstrip()
            if line:
                print(f"[COM4] {line}")
                slave_log.append(line)
        time.sleep(0.1)

    print("[TEST] sending dir to COM5")
    m.write(b"dir\r\n")
    time.sleep(2)
    while m.in_waiting:
        print(f"[COM5] {m.readline().decode('utf-8', errors='replace').rstrip()}")

    print("[TEST] sending play 2 to COM5")
    m.write(b"play 2\r\n")
    time.sleep(1)

    print("[TEST] monitoring COM4 for 40s (BT not required)...")
    end = time.time() + 40
    while time.time() < end:
        time.sleep(0.2)

    stop = True
    time.sleep(0.5)
    sub.close()
    m.close()

    print("--- SUMMARY ---")
    for line in slave_log:
        if any(k in line for k in ["I2S-BENCH", "[PCM]", "I2S tag hb", "build", "drop="]):
            print(line)


if __name__ == "__main__":
    main()
