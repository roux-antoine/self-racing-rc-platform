import glob
import os

flist = glob.glob(os.path.join("kml_files", "*.kml"))
print("Available kml:")
for path in flist:
    print(path)
print()

filename_base = input("Enter filename base: ")

first_time = True
coordinates_line = None
with open(f"kml_files/{filename_base}.kml") as kml_file:
    for line in kml_file:
        if "-122" in line and first_time == False:
            coordinates_line = line.strip()
            break
        if "-122" in line:
            first_time = False
        


with open(f"lat_lon_files/{filename_base}.txt", "w") as out_file:
    for point_str in coordinates_line.split():
        out_file.write(f"{point_str.split(',')[0]} {point_str.split(',')[1]}\n")
