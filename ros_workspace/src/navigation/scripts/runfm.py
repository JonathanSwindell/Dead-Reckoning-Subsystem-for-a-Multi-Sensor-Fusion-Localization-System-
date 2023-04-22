#!/usr/bin/env python3
from __future__ import print_function
"""

Monitors battery and GPS to do measurements
grep "CAPAC"  /sys/class/power_supply/BAT0/uevent
POWER_SUPPLY_CAPACITY=83

"""
import os
import datetime

import rospy

#from gpsread import acquire_gpsdata, start_gpspoller, stop_gpspoller
import time
from geopy.distance import geodesic
from sqlitedict import SqliteDict
import geohash

from gps import *
import threading

# make sure that you do the following on ubuntu: sudo apt-get install gpsd gpsd-clients
# This code is taken from http://www.danmandle.com/blog/getting-gpsd-to-work-with-python/

gpsd = None  # poller daemon , type: gps
gpsp = None  # poller thread , type: GpsPoller


class GpsPoller(threading.Thread):
    """This class implements a thread for collecting GPS data
    """
    def __init__(self):
        threading.Thread.__init__(self)
        global gpsd  # bring it in scope
        gpsd = gps(mode=WATCH_ENABLE)  # starting the stream of info
        self.current_value = None
        self.running = True  # setting the thread running to true

    def run(self):

        global gpsd
        while gpsp.running:
            # this will continue to loop and grab EACH set of gpsd info to
            # clear the buffer
            gpsd.next()


def start_gpspoller():
    """This function starts the GPS polling thread

    Args:
    Returns:
        None
    """
    global gpsp
    if gpsd is None:
        gpsp = GpsPoller()
        gpsp.start()


def stop_gpspoller():
    """This function stops the GPS polling thread

    Args:
    Returns:
        None
    """
    global gpsp
    gpsp.running = False
    gpsp.join()


def acquire_gpsdata():
    """This function starts acquiring GPS data and returns it
    periodically every 0.1 sec

    Args:
    Returns:
        cloc: GPS coordinates
    """
    # cloc = (0, 0)
    if gpsd is None: start_gpspoller()
    while True:
        print("Collecting the gps data")
        if abs(gpsd.fix.latitude) + abs(gpsd.fix.longitude) > 0.001:
            cloc = (gpsd.fix.latitude, gpsd.fix.longitude)
            break
        print("Could not get the gps lock, going to sleep")
        time.sleep(1)  # set to whatever
        print("Waking up from sleep to try again")
    return cloc

def does_geohash_exist(cloc1, cloc2):
    """This function checks if the GPS coordinates are already present in
    fmdb.sqlite

    Args:
        cloc1: GPS coordinate 1
        cloc2: GPS coordinate 2
    Returns:
        Boolean
    """
    with SqliteDict("../data/fmdb.sqlite", autocommit=True) as fmdict:
        ghash1 = geohash.encode(cloc1[0], cloc1[1], precision=8)
        ghash2 = geohash.encode(cloc2[0], cloc2[1], precision=8)
        print("Hash = ", ghash1, ghash2, )
        if ghash1 in fmdict or ghash2 in fmdict:
            print("Exists")
            return True
        else:
            print("Does not exist")
            return False

def start_fm():
    #rospy.init_node('FM_Node', anonymous=True)
    print("Starting the gps poller")
    start_gpspoller()
    print("Started the gps poller")
    totruns = 0
    while rospy.is_shutdown():
        #power = int([line for line in open(
            #"/sys/class/power_supply/BAT0/uevent") if "CAPACI" in line][0].split("=")[1])
        #if power < 30:
            #print("\aPower problem: ", datetime.datetime.now().isoformat())
            #print("Total runs     : ", totruns)
            #break
        totruns += 1
        cloc1 = acquire_gpsdata()
        time.sleep(0.25)
        cloc2 = acquire_gpsdata()
        if geodesic(cloc1, cloc2).miles < 0.0001:
            if not does_geohash_exist(cloc1, cloc2):
                # measure fm data
                print("The geohash does not exist and we are collecting new data")
                os.system("python plotfm.py")

        print("Total distance = ", geodesic(cloc1, cloc2).miles)
        # os.system("python plotfm.py")
    stop_gpspoller()
    
if __name__ == "__main__":
    start_fm()
