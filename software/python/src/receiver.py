import argparse, serial, pandas as pd

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--out", default="imu_data.csv")
    ap.add_argument("--max", type=int, default=50)
    args = ap.parse_args()

    ser = serial.Serial(args.port, args.baud, timeout=1)
    print(f"Listening on {args.port} @ {args.baud}")

    data = []
    while len(data) < args.max:
        line = ser.readline().decode("utf-8", errors="ignore").strip()
        if not line: 
            continue
        if line.startswith("id"):  # Header-Zeile
            continue
        parts = line.split(",")
        if len(parts) == 7:
            n, ax, ay, az, gx, gy, gz = parts
            data.append([int(n), int(ax), int(ay), int(az), int(gx), int(gy), int(gz)])
    ser.close()

    df = pd.DataFrame(data, columns=["n","ax","ay","az","gx","gy","gz"])
    df.to_csv(args.out, index=False)
    print(f"{len(df)} Zeilen gespeichert in {args.out}")

if __name__ == "__main__":
    main()
