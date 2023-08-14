import glob
import os

flist = glob.glob(os.path.join("kml_files", "*.kml"))
print("Available kml:")
for path in flist:
    print(path)
print()

filename_base = input("Enter filename base: ")

coordinates_line = None
with open(f"kml_files/{filename_base}.kml") as kml_file:
    for line in kml_file:
        if "-121" in line:
            coordinates_line = line.strip()
            break


with open(f"lat_lon_files/{filename_base}.txt", "w") as out_file:
    for point_str in coordinates_line.split():
        out_file.write(f"{point_str.split(',')[0]} {point_str.split(',')[1]}\n")
