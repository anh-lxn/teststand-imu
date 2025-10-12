import pandas as pd
import os
import numpy as np

# Pfade
BASE_DIR = os.path.dirname(os.path.abspath(__file__)) # aktueller Ordner
GIT_DIR = os.path.abspath(os.path.join(BASE_DIR, "..", "..", "..")) # teststand-imu Ordner
DATA_DIR = os.path.join(GIT_DIR, "data") # data Ordner
CSV_DATA = os.path.join(DATA_DIR, "data.csv") 

np.set_printoptions(suppress=True, precision=4) # zeigt 0.5680 an, statt 5.6800e-01
df_data = pd.read_csv(CSV_DATA) 
imu_data = df_data.to_numpy() # DataFrame to np Array
