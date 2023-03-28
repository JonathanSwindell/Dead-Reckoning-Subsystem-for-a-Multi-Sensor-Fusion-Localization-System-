import numpy as np
import math
from geopy.distance import geodesic


def group_locations(latitudes, longitudes):
    # create a grid of 5 meter by 5 meter cells

    # https://stackoverflow.com/questions/7477003/calculating-new-longitude-latitude-from-old-n-meters
    
    # The number of kilometers per degree of latitude is approximately the same at all locations, approx
    # (pi/180) * r_earth = 111 km / degree 

    # The number of kilometers per degree of longitude is approximately
    # (pi/180) * r_earth * cos(theta*pi/180) 
    # where theta is the latitude in degrees and r_earth is approximately 6378 km.



    # To get the distance in meters for a 5 meter offset, we can divide 5 meters by the length of one 
    # degree of latitude at the equator (which is approximately 111,320 meters), giving us 0.0000448 degrees of latitude. 
    pi = math.pi
    radius_earth = 6378000 # m
    meters_displacement = 5 # m 
    dy = meters_displacement
    dx = meters_displacement
    
    #                        (((180/pi)/radius_earth) deg / m ) dy = fiveMeterOffset_La (deg)
    
    fiveMeterOffset_La = ((180/pi)/radius_earth) * dy
    fiveMeterOffset_Lo = (((180/pi)/radius_earth) * dx) / math.cos(max(latitudes) * pi/180)
    
    print("5 meter lat offset " + str(fiveMeterOffset_La))
    print("5 meter log offset " + str(fiveMeterOffset_Lo))

    lon_min, lon_max = min(longitudes), max(longitudes)
    lat_min, lat_max = min(latitudes), max(latitudes)
    lon_bins = np.arange(lon_min, lon_max + fiveMeterOffset_Lo, fiveMeterOffset_Lo)
    lat_bins = np.arange(lat_min, lat_max + fiveMeterOffset_La, fiveMeterOffset_La)

    # group locations into cells
    cells = {}
    for i in range(len(longitudes)):
        lon, lat = longitudes[i], latitudes[i]
        cell = (np.digitize(lon, lon_bins), np.digitize(lat, lat_bins))
        if cell not in cells:
            cells[cell] = []
        cells[cell].append((lon, lat))

    return cells

# example usage
# 34.718143, -86.647651 edge of OKT - south west edge of campus
# 34.719184, -86.63336 south east edge of campus
# 34.734527, -86.635936 north east edge of campus
# 34.734879, -86.648617 north west edge of campus

## Longitude test 
# distance between north west and south west points 1.1576mi via https://www.omnicalculator.com/other/latitude-longitude-distance
# 1.1576mi = 1862.97661m; 1862.97661m / 5 = 372.4

# Latitudes test
# distance between north east and south east points 1.0701mi via https://www.omnicalculator.com/other/latitude-longitude-distance
# 1.0701mi = 1722.15901m; 1722.15901m / 5 = 344 ; 


latitudes = [34.718143, 34.719184, 34.734527, 34.734879]
longitudes = [-86.647651, -86.63336, -86.635936, -86.648617]
cells = group_locations(latitudes, longitudes)
print(cells)
