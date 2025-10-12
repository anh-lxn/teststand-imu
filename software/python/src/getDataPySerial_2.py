import os, time, serial, pandas as pd

PORT = "COM5"
BAUD = 115200
DATA_DIR = r"C:\Users\anhle\iCloudDrive\Studium\SHK\teststand-imu\data"
CSV = os.path.join(DATA_DIR, "data.csv")

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)  # Nano booten lassen

print("Fange an Daten zu sammeln...")
ser.write(b"START\n")

rows = []
t0 = time.time()

try:
    while True:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line:
            continue

        parts = line.split(",")
        if len(parts) == 6:
            ax, ay, az, gx, gy, gz = map(float, parts)
            rows.append([ax, ay, az, gx, gy, gz])
            print(parts)
        elif line == "Ende":
            ser.write(b"START\n")
        else:
            print(line)

        # optional: Timeout (z.B. 60s)

except KeyboardInterrupt:
    print("Manuell gestoppt")
finally:
    ser.close()

df = pd.DataFrame(rows, columns=["ax","ay","az","gx","gy","gz"])
os.makedirs(DATA_DIR, exist_ok=True)
df.to_csv(CSV, index=False)
print(f"{len(df)} Zeilen gespeichert in {CSV}")
