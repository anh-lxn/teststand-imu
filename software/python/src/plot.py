import os
import pandas as pd
import matplotlib.pyplot as plt

BASE_DIR = os.path.dirname(os.path.abspath(__file__)) # aktueller Ordner
GIT_DIR = os.path.abspath(os.path.join(BASE_DIR, "..", "..", "..")) # teststand-imu Ordner
DATA_DIR = os.path.join(GIT_DIR, "data") # data Ordner
CSV_ACCEL = os.path.join(DATA_DIR, "imu_accel_data.csv") # CSV für Accel-Daten
CSV_GYRO = os.path.join(DATA_DIR, "imu_gyro_data.csv") # CSV für Gyro-Daten
# CSV laden
df = pd.read_csv(CSV_GYRO)

# nur IDs die mit "gxp" anfangen
df_gxp = df[df["id"].str.startswith("gxp")].copy()

# Messungsnummer extrahieren (Zahl hinter gxp)
df_gxp["num"] = df_gxp["id"].str.extract(r'(\d+)').astype(int)

# Plot gx gegen Nummer
plt.figure(figsize=(10,5))
plt.plot(df_gxp["num"], df_gxp["gy"], marker="o", label="gx (gxp)")

plt.xlabel("Messungsnummer")
plt.ylabel("gx (raw)")
plt.title("Gyroskop gx für gxp1..200")
plt.legend()
plt.grid(True)
plt.show()
