import os
import sys
import serial
import pandas as pd

# === Pfade ===
# os.path.abspath(__file__)      → absoluter Pfad zur Datei
# os.path.dirname(path)          → Verzeichnisname vom Pfad
# os.path.join(a, b, ...)        → Pfade zusammenbauen

BASE_DIR = os.path.dirname(os.path.abspath(__file__)) # aktueller Ordner
GIT_DIR = os.path.abspath(os.path.join(BASE_DIR, "..", "..", "..")) # teststand-imu Ordner
DATA_DIR = os.path.join(GIT_DIR, "data") # data Ordner
CSV_ACCEL = os.path.join(DATA_DIR, "imu_accel_data.csv") # CSV für Accel-Daten
CSV_GYRO = os.path.join(DATA_DIR, "imu_gyro_data.csv") # CSV für Gyro-Daten


# === Einstellungen anpassen ===
PORT = "COM5"
BAUDRATE = 115200

# === Verbindung herstellen ===
ser = serial.Serial(PORT, BAUDRATE, timeout=1) # IDE Code wird nochmal ausgeführt

print("Fange an Daten zu sammeln...")

# Leere Daten Listen
data_accel = []
data_gyro = []

#=== Accel-Daten sammeln ===
while True:
    try:
        line = ser.readline().decode("utf-8").strip() # Lese die Serial
        if not line: # Wenn keine Daten, nächste Iteration
            continue
        # Header-Zeile überspringen
        if line.startswith("id"):  
            continue
        
        parts = line.split(",")
        
        if not "," in line:
            print(line)

        # Accel Data
        if len(parts) == 8:
            id, i, ax, ay, az, gx, gy, gz = parts
            data_accel.append([id, int(i),int(ax), int(ay), int(az), int(gx), int(gy), int(gz)])
        
        
        # Gyro Data
        if len(parts) == 6:
            id, i, axis, gyro_raw, gyro_dps, w = parts
            data_gyro.append([id, i, axis, gyro_raw, gyro_dps, w])

        # Messung Stoppen
        if line == "CALIBRATION DONE":
            break

    except KeyboardInterrupt:
        print("Manuell gestoppt")
        break

ser.close()

# === In DataFrame speichern ===
# Accel Data
df_accel = pd.DataFrame(data_accel, columns=["id","i","ax","ay","az","gx","gy","gz"])
df_accel.to_csv(CSV_ACCEL, index= False)
# Gyro Data
df_gyro = pd.DataFrame(data_gyro, columns=["id","i","axis","gyro_raw", "gyro_dps", "w"])
df_gyro.to_csv(CSV_GYRO, index = False)

print(f"{len(df_accel)} Zeilen gespeichert in {CSV_ACCEL}")
print(f"{len(df_gyro)} Zeilen gespeichert in {CSV_GYRO}")