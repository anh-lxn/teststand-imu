import pandas as pd
import matplotlib.pyplot as plt
import os
import numpy as np

# Pfade
BASE_DIR = os.path.dirname(os.path.abspath(__file__))  # aktueller Ordner
GIT_DIR = os.path.abspath(os.path.join(BASE_DIR, "..", "..", ".."))
DATA_DIR = os.path.join(GIT_DIR, "data")
CSV = os.path.join(DATA_DIR, "data.csv")

# CSV laden
df = pd.read_csv(CSV)
ax = df["ax"]
print(df.index)
# Plot
plt.figure(figsize=(6, 4))
plt.plot(df.index, ax, marker='o')
plt.title("Erste Spalte der Messdaten")
plt.xlabel("Index")
plt.ylabel("Wert (Spalte 1)")
plt.grid(True)

# <<< FESTE Y-ACHSENSKALA >>>
#plt.ylim(0, 12)  # Bereich fixieren
#plt.yticks(np.arange(0, 13, 1))  # z. B. in 1er Schritten

plt.show()