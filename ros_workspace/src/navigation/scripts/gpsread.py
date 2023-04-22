from gps import *
import time
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
        time.sleep(0.1)  # set to whatever
        print("Waking up from sleep to try again")
    return cloc

if __name__=="__main__":
    cloc = acquire_gpsdata()
    print("The location is: ",cloc)