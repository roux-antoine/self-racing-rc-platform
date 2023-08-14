import glob
import os

import numpy as np
import utm

flist = glob.glob(os.path.join("lat_lon_files", "*.txt"))
print("Available txt: ")
for path in flist:
    print(path)
print()

basename = input("Enter basename for lat_lon file: ")

with open(f"lat_lon_files/{basename}.txt") as lat_lon_file:
    longitudes_latitudes = [
        [float(line.split()[0]), float(line.split()[1])]
        for line in lat_lon_file.readlines()
    ]

longitudes = np.array(np.array(longitudes_latitudes)[:, 0])
latitudes = np.array(np.array(longitudes_latitudes)[:, 1])

utm_values = utm.from_latlon(np.array(latitudes), np.array(longitudes))

with open(f"x_y_files/{basename}.txt", "w") as out_file:
    for x, y in zip(utm_values[0], utm_values[1]):
        out_file.write(f"{x} {y}\n")
