import os

import matplotlib.pyplot as plt
import shapefile as shp


class NMEA:
    def __init__(self, sentence: str):
        self.sentence = sentence
        self.latitude = self.compute_latitude()
        self.longitude = self.compute_longitude()

    def compute_latitude(self):
        latitude_raw = self.sentence.split(",")[3]
        latitude_degree = latitude_raw[0:2]
        latitude_minutes = latitude_raw[2:]
        latitude_sign = "+" if self.sentence.split(",")[4] == "N" else "-"
        latitude_decimal_unsigned = (
            float(latitude_degree) + float(latitude_minutes) / 60
        )
        latitude_decimal = float(f"{latitude_sign}{latitude_decimal_unsigned}")

        return latitude_decimal

    def compute_longitude(self):
        longitude_raw = self.sentence.split(",")[5]
        longitude_degree = longitude_raw[0:3]
        longitude_minutes = longitude_raw[3:]
        longitude_sign = "+" if self.sentence.split(",")[6] == "E" else "-"
        longitude_decimal_unsigned = (
            float(longitude_degree) + float(longitude_minutes) / 60
        )
        longitude_decimal = float(f"{longitude_sign}{longitude_decimal_unsigned}")

        return longitude_decimal


laguna_seca_boundaries = {
    "min_lat": 36.5789,
    "max_lat": 36.59,
    "min_lon": -121.759,
    "max_lon": -121.748,
}
laguna_seca_basepath = "maps/laguna-seca-osmaxx_wgs-84_2022-09-03_shapefile_full-detail/data/laguna-seca-osmaxx_wgs-84_2022-09-03_shapefile_full-detail"

rex_manor_boundaries = {
    "min_lat": 37.402,
    "max_lat": 37.407,
    "min_lon": -122.087,
    "max_lon": -122.082,
}
rex_manor_basepath = "maps/rex-manor_wgs-84_2022-09-03_shapefile_full-detail/data/rex-manor_wgs-84_2022-09-03_shapefile_full-detail"

color_dict = {
    "misc_l.shp": "k",
    "building_a.shp": "b",
    "road_l.shp": "g",
    "landmass_a.shp": "y",
}

BASEPATH = rex_manor_basepath
BOUNDARIES = rex_manor_boundaries

filenames = os.listdir(BASEPATH)
for filename in filenames:
    if filename in color_dict.keys():
        sf = shp.Reader(os.path.join(BASEPATH, filename))
        for (
            shape
        ) in (
            sf.shapeRecords()
        ):  # TODO see if there are any other stuff on top of shapeRecords
            record = shape.record  # TODO maybe we can extract more info from there
            x = [i[0] for i in shape.shape.points[:]]
            y = [i[1] for i in shape.shape.points[:]]
            plt.plot(x, y, color=color_dict[filename])


# read NMEAs

nmeas = []

RECORDING = "gps_recordings/recordings_biking_and_garden/bike_with_antenna_10Hz.nmea"

with open(RECORDING) as logfile:
    for line in logfile.readlines():
        if "GNRMC" in line and line.split(",")[2] == "A":
            nmeas.append(NMEA(line))

latitudes = [x.latitude for x in nmeas]
longitudes = [x.longitude for x in nmeas]

# plot everything

plt.scatter(longitudes, latitudes)
plt.xlim(BOUNDARIES["min_lon"], BOUNDARIES["max_lon"])
plt.ylim(BOUNDARIES["min_lat"], BOUNDARIES["max_lat"])
plt.show()
