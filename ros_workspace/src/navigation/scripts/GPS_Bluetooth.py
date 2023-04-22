#!/usr/bin/env python3
import logging.config
import time
import math
import logging
import logging.config
import json # Uses JSON package
import _pickle as pickle # Serializing and de-serializing a Python object structure
from bluetooth import * # Python Bluetooth library
#from navigation.LongLatConversions import sphere_longlat_to_m

from itertools import combinations

import rospy
from navigation.msg import gps
from navigation.msg import reset

BAUD_RATE = 115200


current_location = ""

logger = logging.getLogger('bleServerLogger')
def sphere_longlat_to_m(long0, lat0, long1, lat1): # X is N/S, Y is E/W.
    r = 6_731_000  # meters
    dy = ((long1-long0)/360.0) * (math.pi*2*r)
    dx = ((lat1-lat0)/360.0) * (math.pi*2*r)
    return dx, dy

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
    dX = dX_r * math.sqrt(2-2*math.cos(phi1-phi0)) #Law of cosines
    if (lat1 < lat0):
        dX = dX * -1
    return dX, dY

def reset_window(window): # this function currently doesn't work
    for x in window:
        x = 0
    
def test_still(lat_wnd, lon_wnd):
    comb = combinations([1, 2, 3], 2)
    avg = 0.0
    length = 0
    for index in list(comb):
        length = length + 1
        dx, dy = sphere_longlat_to_m(lat_wnd[index[0]],lon_wnd[index[0]], lat_wnd[index[1]],lon_wnd[index[1]])
        avg = avg + math.sqrt(dx*dx + dy*dy)
        
    return avg / length

def startLogging(
    default_path='configLogger.json',
    default_level=logging.INFO,
    env_key='LOG_CFG'
):
    # Setup logging configuration
    path = default_path
    value = os.getenv(env_key, None)
    if value:
        path = value
    if os.path.exists(path):
        with open(path, 'rt') as f:
            config = json.load(f)
        logging.config.dictConfig(config)
    else:
        logging.basicConfig(level=default_level)

class bleServer:
    def __init__(self, serverSocket=None, clientSocket=None):
        if serverSocket is None:
            self.dataObj = None
            self.serverSocket = serverSocket
            self.clientSocket = clientSocket
            self.serviceName="BluetoothServices"
            self.jsonFile ="text.json"
            self.uuid = "fa87c0d0-afac-11de-8a39-0800200c9a66"
        else:
            self.serverSocket = serverSocket
            self.clientSocket = clientSocket

    def getBluetoothSocket(self):
        try:
            self.serverSocket=BluetoothSocket( RFCOMM )
            rospy.loginfo("Bluetooth server socket successfully created for RFCOMM service...")
        except (BluetoothError, SystemExit, KeyboardInterrupt) as e:
            rospy.logwarn("Failed to create the bluetooth server socket ", exc_info=True)

    def getBluetoothConnection(self):
        try:
            self.serverSocket.bind(("",PORT_ANY))
            rospy.loginfo("Bluetooth server socket bind successfully on host "" to PORT_ANY...")
        except (Exception, BluetoothError, SystemExit, KeyboardInterrupt) as e:
            rospy.logwarn("Failed to bind server socket on host to PORT_ANY ... ", exc_info=True)
        try:
            self.serverSocket.listen(1)
            rospy.loginfo("Bluetooth server socket put to listening mode successfully ...")
        except (Exception, BluetoothError, SystemExit, KeyboardInterrupt) as e:
            rospy.logwarn("Failed to put server socket to listening mode  ... ", exc_info=True)
        try:
            port=self.serverSocket.getsockname()[1]
            rospy.loginfo("Waiting for connection on RFCOMM channel %d" % (port))
        except (Exception, BluetoothError, SystemExit, KeyboardInterrupt) as e:
            rospy.logwarn("Failed to get connection on RFCOMM channel  ... ", exc_info=True)

    def advertiseBluetoothService(self):
        try:
            advertise_service( self.serverSocket, self.serviceName,
                            service_id = self.uuid,
                            service_classes = [ self.uuid, SERIAL_PORT_CLASS ],
                            profiles = [ SERIAL_PORT_PROFILE ],
        #                   protocols = [ OBEX_UUID ]
                            )
            rospy.loginfo("%s advertised successfully ..." % (self.serviceName))
        except (Exception, BluetoothError, SystemExit, KeyboardInterrupt) as e:
            rospy.logwarn("Failed to advertise bluetooth services  ... ", exc_info=True)

    def acceptBluetoothConnection(self):
        try:
            self.clientSocket, clientInfo = self.serverSocket.accept()
            rospy.loginfo("Accepted bluetooth connection from %s", clientInfo)
        except (Exception, BluetoothError, SystemExit, KeyboardInterrupt) as e:
            rospy.logwarn("Failed to accept bluetooth connection ... ", exc_info=True)

    def recvData(self):
        try:
            while True:
                data= self.clientSocket.recv(1024)
                if not data:
                    self.clientSocket.send("EmptyBufferResend")
                # remove the length bytes from the front of buffer
                # leave any remaining bytes in the buffer!
                dataSizeStr, ignored, data = data.partition(':')
                dataSize = int(dataSizeStr)
                if len(data) < dataSize:
                    self.clientSocket.send("CorruptedBufferResend")
                else:
                    self.clientSocket.send("DataRecived")
                    break
            rospy.loginfo("Data received successfully over bluetooth connection")
            return data
        except (Exception, IOError, BluetoothError) as e:
            pass

    def deserializedData(self, _dataRecv):
        try:
            # print(_dataRecv)
            self.dataObj=pickle.loads(_dataRecv)
            rospy.loginfo("Serialized string converted successfully to object")
        except (Exception, pickle.UnpicklingError) as e:
            rospy.logwarn("Failed to de-serialized string ... ", exc_info=True)
            # print("Oh NO!!")

    def writeJsonFile(self):
        try:
            # Open a file for writing
            jsonFileObj = open(self.jsonFile,"w")
            rospy.loginfo("%s file successfully opened to %s" % (self.jsonFile, jsonFileObj))
            # Save the dictionary into this file
            # (the 'indent=4' is optional, but makes it more readable)
            json.dump(self.dataObj,jsonFileObj, indent=4)
            rospy.loginfo("Content dumped successfully to the %s file" %(self.jsonFile))
            # Close the file
            jsonFileObj.close()
            rospy.loginfo("%s file successfully closed" %(self.jsonFile))
        except (Exception, IOError) as e:
            rospy.logwarn("Failed to write json contents to the file ... ", exc_info=True)

    def closeBluetoothSocket(self):
        try:
            self.clientSocket.close()
            self.serverSocket.close()
            rospy.loginfo("Bluetooth sockets successfully closed ...")
        except (Exception, BluetoothError) as e:
            rospy.logwarn("Failed to close the bluetooth sockets ", exc_info=True)

    def start(self):
            # Create the server socket
            self.getBluetoothSocket()
            # get bluetooth connection to port # of the first available
            self.getBluetoothConnection()
            # advertising bluetooth services
            self.advertiseBluetoothService()
            # Accepting bluetooth connection
            self.acceptBluetoothConnection()
            
            
    def receive(self, publisher, rst_publisher):
            stand_still_cnt = 0 # If it gets to 5, send event to reset dead reckoning
            lat_window = [0, 0, 0, 0]
            lon_window = [0, 0, 0, 0]
            prev_lat = 30
            prev_lon = -80
            # receive data
            dataRecv=self.recvData()
            print("received " + str(dataRecv))
            # de-serializing data
            self.deserializedData(dataRecv)
            # Writing json object to the file
            self.writeJsonFile()
            
            # Used to see if the user has remained still:
            
            while not rospy.is_shutdown():
                try:
                    data = self.clientSocket.recv(1024)
                    if not data:
                        break
                    gps_msg = gps()
                    print("Received:", data.decode())
                    try:
                        message = data.decode().split(',')
                        print("BEFORE FLOAT ", message[0], " , ",  message[1]) 
                        lat = float(message[0])
                        lon = float(message[1])
                        print("AFTER FLOAT ", lat, " , ", lon)
                        gps_msg.lat = lat
                        gps_msg.lon = lon
                        lat_window.insert(0, lat)
                        lon_window.insert(0,lon)
                        lat_window.pop()
                        lon_window.pop()
                        dx, dy = longlat_to_meters(prev_lon, prev_lat, lon, lat)
                        #print("SPHERE: ", sphere_longlat_to_m(prev_lon, prev_lat, lon, lat))
                        #print("ELLIPSOIDAL: ", longlat_to_meters(prev_lon, prev_lat, lon, lat))
                        print(dx+dy)
                        print(test_still(lat_window, lon_window))
                        if (dx + dy) < 0.5: # You have not moved over 0.25 metersish
                            stand_still_cnt = stand_still_cnt + 1
                            if stand_still_cnt > 4:
                                stand_still_cnt = 0
                                rst_msg = reset()
                                rst_msg.newLat = lat;
                                rst_msg.newLong = lon;
                                #rst_publisher.publish(rst_msg)
                                reset_window(lat_window)
                                reset_window(lon_window)
                                rospy.loginfo("Sending reset message!")
                        else:
                            stand_still_cnt = 0
                        prev_lon = lon
                        prev_lat = lat
                        publisher.publish(gps_msg)
                        # Put checks here to see if you are stopped
                    except Exception as e:
                        print(e)
                        rospy.logwarn("Error parsing GPS message")
                except (Exception, IOError) as e:
                    print("Bluetooth connection error:", e)
                    break
                #rospy.spin()

    def stop(self):
        # Disconnecting bluetooth sockets
        self.closeBluetoothSocket()


def blueSvrWork():
    rospy.loginfo("Gps Node launched")
    gps_publisher = rospy.Publisher('gps_data', gps, queue_size=10)
    reset_publisher = rospy.Publisher('reset_bno', reset, queue_size=10)
    rospy.init_node('GPS_Bluetooth', anonymous=True)
    startLogging()
    bleSvr = bleServer()
    bleSvr.start()
    bleSvr.receive(gps_publisher, reset_publisher)
    bleSvr.stop()

if __name__ == '__main__':
    try:
        blueSvrWork()
    except rospy.ROSInterruptException:
        pass
