import pandas as pd
import numpy as np

# Read in CSV File 
csv = pd.read_csv("April3CombinedCampusLoop.csv")
output = csv.iloc[:,6:8]

## CHANGE BASED ON WHETHER ORIENTATION WAS MADE CORRECTLY 
output.columns = ["Y","X"]

# Add columns for calculating long lat 
output = output.assign(nLat=np.nan, nLng=np.nan)
output.iloc[:1,-2:] = csv.iloc[:1,-2:] 

import math

# Meters to Long Lat Method
def meters_to_longlat(dx, dy, long0, lat0): # X is N/S, Y is E/W.
    a = 6_378_137  # Semi-major axis for WGS-84
    b = 6_356_732.3142  # Semi-minor axis
    ab = a * b
    a2 = a * a
    b2 = b * b
    phi0 = lat0 * (math.pi / 180)
    try: 
        latitudal_radius = ab * math.sqrt(1 / b2 + a2 * math.tan(phi0))
    except ValueError:
        print('ValueError: phi0:', phi0)
        print('lat0:', lat0)
        print('ab:', ab)
        print('b2 + a2', (b2+a2))
        print('math.tan(phi0):', math.tan(phi0))          
        raise
    latitudal_radius = ab * math.sqrt(1 / abs(b2 + a2 * math.tan(phi0)))
    delta_longitude = (dy / (2*math.pi*latitudal_radius)) * 360 # Convert to degrees
    eclipse_r = (ab / math.sqrt(a2 * math.pow(math.sin(phi0), 2) + b2 * math.pow(math.cos(phi0), 2)))  # Find radius of ellipse at given latitude. Essentially the distance from the center of the earth.
    delta_latitude = math.acos(1 - ((dx*dx)/(2*eclipse_r*eclipse_r))) * (180/math.pi) # Law of cosines.
    long1 = long0 + delta_longitude
    lat1 = lat0
    if (dx < 0):
        lat1 = lat1 - delta_latitude
    else:
        lat1 = lat1 + delta_latitude

    return long1, lat1

# Calculation Station
for i in range(1, len(output)):
    dx = output.loc[i, 'X']
    dy = output.loc[i, 'Y']
    long0 = output.loc[i-1, 'nLng']
    lat0 = output.loc[i-1, 'nLat']
    long1, lat1 = meters_to_longlat(dx, dy, long0, lat0)
    output.loc[i, 'nLng'] = long1
    output.loc[i, 'nLat'] = lat1


# Output to CSV
csv_update = pd.concat([csv,output.iloc[:,-2:]],axis=1)
csv_update.to_csv('csv_update.csv',index=False)