import os
import pandas as pd
import matplotlib.pyplot as plt
from enum import StrEnum

# Pfade
BASE_DIR = os.path.dirname(os.path.abspath(__file__)) # aktueller Ordner
GIT_DIR = os.path.abspath(os.path.join(BASE_DIR, "..", "..", "..")) # teststand-imu Ordner
DATA_DIR = os.path.join(GIT_DIR, "data") # data Ordner
CSV_ACCEL = os.path.join(DATA_DIR, "imu_accel_data.csv") # CSV für Accel-Daten
CSV_GYRO = os.path.join(DATA_DIR, "imu_gyro_data.csv") # CSV für Gyro-Daten

# Variablen
g0 = 9.80665 # m/s²
counts_per_g = 16384.0  # laut Datenblatt
counts_per_dps = 131.0  # laut Datenblatt

# Dataframes
df_accel = pd.read_csv(CSV_ACCEL)
df_gyro = pd.read_csv(CSV_GYRO)

# Dataframe Anpassungen
# Vorzeichen der Winkelgeschwindigkeiten korrigieren
df_gyro.loc[df_gyro["id"].str.contains("n"), "w"] *= -1

# Sortieren
df_gx = df_gyro[df_gyro["id"].isin(["gxn", "gxp"])] # Dataframe mit nur rohen Gyrodaten vom X
df_gy = df_gyro[df_gyro["id"].isin(["gyn", "gyp"])] # Dataframe mit nur rohen Gyrodaten vom Y
df_gz = df_gyro[df_gyro["id"].isin(["gzn", "gzp"])] # Dataframe mit nur rohen Gyrodaten vom Z


# Sollwerte Accelerometer in g für jede Phase
ax_expected_g_dict = {"axp": +1, "axn": -1, "ayp":  0, "ayn":  0, "azp":  0, "azn":  0}
ay_expected_g_dict = {"axp": 0, "axn": 0, "ayp":  1, "ayn":  -1, "azp":  0, "azn":  0}
az_expected_g_dict = {"axp": 0, "axn": 0, "ayp":  0, "ayn":  0, "azp":  1, "azn":  -1}

# Sollwerte Gyroskop in °/s
gx_expected_deg = df_gx["w"]
gy_expected_deg = df_gy["w"]
gz_expected_deg = df_gz["w"]


# Accel-Sollwerte für die Phasen
df_accel_expected = df_accel.copy() 
for i in range(len(df_accel["id"])):
    id = df_accel["id"].values[i]                           # aktuelle ID z.B. axp, axn, ayp, ...
    df_accel_expected["ax"].values[i] = ax_expected_g_dict[id]   # ersetzt den accel-wert mit dem erwartungswert für Position X
    df_accel_expected["ay"].values[i] = ay_expected_g_dict[id]   # ersetzt den accel-wert mit dem erwartungswert für Position Y
    df_accel_expected["az"].values[i] = az_expected_g_dict[id]   # ersetzt den accel-wert mit dem erwartungswert für Position Z
ax_expected_g = df_accel_expected["ax"]
ay_expected_g = df_accel_expected["ay"]
az_expected_g = df_accel_expected["az"]




# Accel Datenauswertung
# Mittelwerte 
mean_accel = df_accel.groupby("id")[["ax", "ay", "az"]].mean().round(2) # Mittel für alle 6 Positionen axp, axn, ayp, ayn, azp, azn
mean_gyro_idle = df_accel.groupby("id")[["gx", "gy", "gz"]].mean().round(2) # Mittel für alle 6 Positionen -> Gyroskop ist in Ruhelage
mean_gyro = df_gyro.groupby("id")[["gyro_raw", "gyro_dps", "w"]].mean().round(2)

# Kovarianz und Varianz
cov_gx = df_gx[["w","gyro_raw"]].cov().loc["w","gyro_raw"]
cov_gy = df_gy[["w","gyro_raw"]].cov().loc["w","gyro_raw"]
cov_gz = df_gz[["w","gyro_raw"]].cov().loc["w","gyro_raw"]

var_gx = df_gx["w"].var()
var_gy = df_gy["w"].var()
var_gz = df_gz["w"].var()

# lineare Regression y = k*x + b
k_gx = cov_gx/var_gx
k_gy = cov_gy/var_gy
k_gz = cov_gz/var_gz

b_gx = df_gx["gyro_raw"].mean() - k_gx * df_gx["w"].mean()
b_gy = df_gy["gyro_raw"].mean() - k_gy * df_gy["w"].mean()
b_gz = df_gz["gyro_raw"].mean() - k_gz * df_gz["w"].mean()

# Bias
bias_ax = (mean_accel.loc["axp", "ax"] + mean_accel.loc["axn", "ax"])/2
bias_ay = (mean_accel.loc["ayp", "ay"] + mean_accel.loc["ayn", "ay"])/2
bias_az = (mean_accel.loc["azp", "az"] + mean_accel.loc["azn", "az"])/2

bias_gx_idle = mean_gyro_idle["gx"].mean().round(2) 
bias_gy_idle = mean_gyro_idle["gy"].mean().round(2)
bias_gz_idle = mean_gyro_idle["gz"].mean().round(2)

bias_gx = b_gx
bias_gy = b_gy
bias_gz = b_gz

# Counts
counts_per_ax = (mean_accel.loc["axp", "ax"] - mean_accel.loc["axn", "ax"])/2
counts_per_ay = (mean_accel.loc["ayp", "ay"] - mean_accel.loc["ayn", "ay"])/2
counts_per_az = (mean_accel.loc["azp", "az"] - mean_accel.loc["azn", "az"])/2

counts_per_gx = k_gx
counts_per_gy = k_gy
counts_per_gz = k_gz

scale_g_ax = 1/counts_per_ax
scale_g_ay = 1/counts_per_ay
scale_g_az = 1/counts_per_az

scale_deg_gx = 1/counts_per_gx
scale_deg_gy = 1/counts_per_gy
scale_deg_gz = 1/counts_per_gz

"""
# Scales ((m/s²)/Counts)
scale_ms2_ax = g0/counts_per_ax
scale_ms2_ay = g0/counts_per_ay
scale_ms2_az = g0/counts_per_az
"""

# Kalibrierte Werte als Beispiel zum Plotten
# a = (a_raw - a_bias) * a_scale
ax_cal_g = (df_accel["ax"] - bias_ax) * scale_g_ax
ay_cal_g = (df_accel["ay"] - bias_ay) * scale_g_ay
az_cal_g = (df_accel["az"] - bias_az) * scale_g_az

# g = (g_raw - g_bias) * g_scale
gx_cal_deg = (df_gx["gyro_raw"] - bias_gx) * scale_deg_gx
gy_cal_deg = (df_gy["gyro_raw"] - bias_gy) * scale_deg_gy
gz_cal_deg = (df_gz["gyro_raw"] - bias_gz) * scale_deg_gz


# Rohwerte
ax_raw_g = df_accel["ax"] / counts_per_g
ay_raw_g = df_accel["ay"] / counts_per_g
az_raw_g = df_accel["az"] / counts_per_g

gx_raw_deg = df_gx["gyro_dps"]
gy_raw_deg = df_gy["gyro_dps"]
gz_raw_deg = df_gz["gyro_dps"]


def plot_accelerometer():
    # Plot
    figure, (ax_plot, ay_plot, az_plot) = plt.subplots(3, 1, figsize=(8, 10))

    # Accelerometerwerte für X
    ax_plot.scatter(df_accel.index+1, ax_expected_g, s=0.3, label="ax_expected", color="red")
    ax_plot.scatter(df_accel.index+1, ax_raw_g, s=0.3, label="ax_raw", color="orange")
    ax_plot.scatter(df_accel.index+1, ax_cal_g, s=0.3, label="ax_cal", color="green")
    ax_plot.legend()
    ax_plot.set_title("Accelerometerwerte für X")
    ax_plot.set_xlabel("Messungen")
    ax_plot.set_ylabel("ax")

    # Accelerometerwerte für Y
    ay_plot.scatter(df_accel.index+1, ay_expected_g, s=0.3, label="ay_expected", color="red")
    ay_plot.scatter(df_accel.index+1, ay_raw_g, s=0.3, label="ay_raw", color="orange")
    ay_plot.scatter(df_accel.index+1, ay_cal_g, s=0.3, label="ay_cal", color="green")
    ay_plot.legend()
    ay_plot.set_title("Accelerometerwerte für Y")
    ay_plot.set_xlabel("Messungen")
    ay_plot.set_ylabel("ay")

    # Accelerometerwerte für Z
    az_plot.scatter(df_accel.index+1, az_expected_g, s=0.3, label="az_expected", color="red")
    az_plot.scatter(df_accel.index+1, az_raw_g, s=0.3, label="az_raw", color="orange")
    az_plot.scatter(df_accel.index+1, az_cal_g, s=0.3, label="az_cal", color="green")
    az_plot.legend()
    az_plot.set_title("Accelerometerwerte für Z")
    az_plot.set_xlabel("Messungen")
    az_plot.set_ylabel("az")


    plt.tight_layout()
    plt.show()

def plot_gyroskop():
     # Plot
    figure, (gx_plot, gy_plot, gz_plot) = plt.subplots(3, 1, figsize=(8, 10))

    # Gyroskopwerte für Drehung um X-Achse
    gx_plot.scatter(df_gx.index+1, gx_expected_deg, s=0.3, label="gx_expected", color="red")
    gx_plot.scatter(df_gx.index+1, gx_raw_deg, s=0.3, label="gx_raw", color="orange")
    gx_plot.scatter(df_gx.index+1, gx_cal_deg, s=0.3, label="gx_cal", color="green")
    gx_plot.legend()
    gx_plot.set_title("Gyroskopwerte für Drehung um X-Achse")
    gx_plot.set_xlabel("Messungen")
    gx_plot.set_ylabel("gx")

    # Gyroskopwerte für Drehung um Y-Achse
    gy_plot.scatter(df_gy.index+1, gy_expected_deg, s=0.3, label="gy_expected", color="red")
    gy_plot.scatter(df_gy.index+1, gy_raw_deg, s=0.3, label="gy_raw", color="orange")
    gy_plot.scatter(df_gy.index+1, gy_cal_deg, s=0.3, label="gy_cal", color="green")
    gy_plot.legend()
    gy_plot.set_title("Gyroskopwerte für Drehung um Y-Achse")
    gy_plot.set_xlabel("Messungen")
    gy_plot.set_ylabel("gy")

    # Gyroskopwerte für Drehung um Z-Achse
    gz_plot.scatter(df_gz.index+1, gz_expected_deg, s=0.3, label="gz_expected", color="red")
    gz_plot.scatter(df_gz.index+1, gz_raw_deg, s=0.3, label="gz_raw", color="orange")
    gz_plot.scatter(df_gz.index+1, gz_cal_deg, s=0.3, label="gz_cal", color="green")
    gz_plot.legend()
    gz_plot.set_title("Gyroskopwerte für Drehung um Z-Achse")
    gz_plot.set_xlabel("Messungen")
    gz_plot.set_ylabel("gz")

    plt.tight_layout()
    plt.show()

def print_results():
    print("=== Ergebnisse ===")
    print(f"Bias Accelerometer: ax: {bias_ax}, ay: {bias_ay}, az: {bias_az}")
    print(f"Bias Gyroskop (idle): gx: {bias_gx_idle}, gy: {bias_gy_idle}, gz: {bias_gz_idle}")
    print(f"Bias Gyroskop: gx: {bias_gx}, gy: {bias_gy}, gz: {bias_gz}")
    print(f"Counts per g: ax: {counts_per_ax}, ay: {counts_per_ay}, az: {counts_per_az}")
    print(f"Counts per °/s: gx: {counts_per_gx}, gy: {counts_per_gy}, gz: {counts_per_gz}")
    print(f"Scale Accelerometer (g/Count): ax: {scale_g_ax}, ay: {scale_g_ay}, az: {scale_g_az}")
    print(f"Scale Gyroskop (°/s/Count): gx: {scale_deg_gx}, gy: {scale_deg_gy}, gz: {scale_deg_gz}")

#print_results()
plot_accelerometer()
plot_gyroskop()