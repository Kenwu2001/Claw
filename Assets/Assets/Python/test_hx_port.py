import serial, time

PORT = "COM7"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(1)

print("Start reading... Ctrl+C to stop")
while True:
    line = ser.readline().decode(errors="ignore").strip()
    if not line:
        continue
    print("RAW:", line)
