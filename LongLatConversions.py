import math

def sphere_m_to_longlat(dx,dy,long0,lat0): # X is N/S, Y is E/W.
    r = 6_731_000 # meters
    d_lat = dy/(2*math.pi*r) * 360
    d_long = dx/(2*math.pi*r) * 360
    long1 = long0
    lat1 = lat0
    print("delta_longitude: ", d_long)
    print("delta_latitude: ", d_lat)
    if (dy < 0):
        long1 = long1 - d_long
    else:
        long1 = long1 + d_long

    if (dx < 0):
        lat1 = lat1 + d_lat
    else:
        lat1 = lat1 + d_lat

    return long1, lat1

def sphere_longlat_to_m(long0, lat0, long1, lat1): # X is N/S, Y is E/W.
    r = 6_731_000  # meters
    dy = ((long1-long0)/360.0) * (math.pi*2*r)
    dx = ((lat1-lat0)/360.0) * (math.pi*2*r)
    return dx, dy



def meters_to_longlat(dx, dy, long0, lat0): # X is N/S, Y is E/W.
    a = 6_378_137  # Semi-major axis for WGS-84
    b = 6_356_732.3142  # Semi-minor axis
    ab = a * b
    a2 = a * a
    b2 = b * b
    phi0 = lat0 * (math.pi / 180)
    latitudal_radius = ab * math.sqrt(1 / (b2 + a2 * math.tan(phi0)))
    delta_longitude = (dy / (2*math.pi*latitudal_radius)) * 360 # Convert to degrees
    eclipse_r = (ab / math.sqrt(a2 * math.pow(math.sin(phi0), 2) + b2 * math.pow(math.cos(phi0), 2)))  # Find radius of ellipse at given latitude. Essentially the distance from the center of the earth.
    delta_latitude = math.acos(1 - ((dx*dx)/(2*eclipse_r*eclipse_r))) * (180/math.pi) # Law of cosines.
    long1 = long0 + delta_longitude
    lat1 = lat0
    #print("delta_longitude: ", delta_longitude)
    #print("delta_latitude: ", delta_latitude)
    if (dx < 0):
        lat1 = lat1 - delta_latitude
    else:
        lat1 = lat1 + delta_latitude

    return long1, lat1

def longlat_to_meters(long0, lat0, long1, lat1): # X is N/S, Y is E/W. Z is altitude
    a = 6_378_137 # Semi-major axis for WGS-84
    b = 6_356_732.3142 #Semi-minor axis
    ab = a * b
    a2 = a * a
    b2 = b * b
    theta = (long1 * (math.pi / 180)) - (long0 * (math.pi / 180))  # Change in longitude in rad. (Longitude final - longitude initial)
    phi0 = lat0 * (math.pi / 180)
    phi1 = lat1 * (math.pi / 180)
    dY = theta * ab * math.sqrt(1/(b2 + a2*math.tan(phi0))) # dX = r*theta, r = radius of equatorial line at given latitude

    dX_r = (ab / math.sqrt(a2*math.pow(math.sin(phi0),2) + b2*math.pow(math.cos(phi0),2))) #Find radius of ellipse at given latitude
    print("hi1", dX_r)
    dX = dX_r * math.sqrt(2-2*math.cos(phi1-phi0)) #Law of cosines
    if (lat1 < lat0):
        dX = dX * -1
    return dX, dY


def test_longlat_to_meters():
    # lat, long
    # 34.730449, -86.638030
    # 34.722436, -86.640885

    #long, lat
    # -86.638030, 34.730449
    # -86.640885, 34.722436
    dx, dy = longlat_to_meters(-86.638030, 34.730449, -86.640885, 34.722436)
    print("WSG-84", dx, dy)
    print("sphere", sphere_longlat_to_m(-86.638030, 34.730449, -86.640885, 34.722436))
    #meters_to_longlat(dx, dy, long0, lat0)
    print("WSG-84", meters_to_longlat(dx, dy, -86.638030, 34.730449))
    print("sphere", sphere_m_to_longlat(dx, dy, -86.638030, 34.730449))




if __name__ == "__main__":
    test_longlat_to_meters()