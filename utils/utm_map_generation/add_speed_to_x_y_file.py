import glob
import os

import numpy as np

flist = glob.glob(os.path.join("x_y_files", "*.txt"))
print("Available txt: ")
for path in flist:
    print(path)
print()

basename = input("Enter basename for x_y file: ")
speed = input("Enter speed: ")

with open(f"x_y_files/{basename}.txt") as lat_lon_file:
    x_y = [
        [float(line.split()[0]), float(line.split()[1])]
        for line in lat_lon_file.readlines()
    ]

x = np.array(np.array(x_y)[:, 0])
y = np.array(np.array(x_y)[:, 1])

with open(f"x_y_files/{basename}_speed.txt", "w") as out_file:
    for (x, y) in x_y:
        out_file.write(f"{x} {y} {speed}\n")
