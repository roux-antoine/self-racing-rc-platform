# How to generate a map

## Generating a set of points in Google Earth

* Create a new folder in the My Places folder
* Click on this new folder
* Click on the New Path button at the top
* Click on the points of interest
* Save the path
* Right click on the path in the My Places tab
* Click Save Place As
* Export as kml

## Converting the kml into a txt file of latitudes and longitudes

* Put the kml in `utils/utm_map_generation/kml_files`
* Use `kml_to_lat_lon.py`

## Converting the lat_lon file into a txt file of utm coordinates

* Use `lat_lon_to_x_y.py`

## Combining the different files if needed

The maps consist of two files:
* a file with the waypoints that the car will follow
* a file with the edges (for easier visualization)

So, if needed, multiple txt files with the x and y coordinates can be combined as follows:
* a file called `foo_waypoints.txt` with all the waypoints
* a file called `foo_edges.txt` with the edges and a space between each set fo edges

# How to plot a map

Use `utm_plotter_from_x_y.py`
