import BNO_Interface
import logging.config
import threading
import time
import logging
import logging.config
import json # Uses JSON package
import _pickle as pickle # Serializing and de-serializing a Python object structure
from bluetooth import * # Python Bluetooth library

BNO_PORT = "COM6"
BAUD_RATE = 115200

current_location = ""
current_BNO_data = ""

logger = logging.getLogger('bleServerLogger')

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
            logger.info("Bluetooth server socket successfully created for RFCOMM service...")
        except (BluetoothError, SystemExit, KeyboardInterrupt) as e:
            logger.error("Failed to create the bluetooth server socket ", exc_info=True)

    def getBluetoothConnection(self):
        try:
            self.serverSocket.bind(("",PORT_ANY))
            logger.info("Bluetooth server socket bind successfully on host "" to PORT_ANY...")
        except (Exception, BluetoothError, SystemExit, KeyboardInterrupt) as e:
            logger.error("Failed to bind server socket on host to PORT_ANY ... ", exc_info=True)
        try:
            self.serverSocket.listen(1)
            logger.info("Bluetooth server socket put to listening mode successfully ...")
        except (Exception, BluetoothError, SystemExit, KeyboardInterrupt) as e:
            logger.error("Failed to put server socket to listening mode  ... ", exc_info=True)
        try:
            port=self.serverSocket.getsockname()[1]
            logger.info("Waiting for connection on RFCOMM channel %d" % (port))
        except (Exception, BluetoothError, SystemExit, KeyboardInterrupt) as e:
            logger.error("Failed to get connection on RFCOMM channel  ... ", exc_info=True)

    def advertiseBluetoothService(self):
        try:
            advertise_service( self.serverSocket, self.serviceName,
                            service_id = self.uuid,
                            service_classes = [ self.uuid, SERIAL_PORT_CLASS ],
                            profiles = [ SERIAL_PORT_PROFILE ],
        #                   protocols = [ OBEX_UUID ]
                            )
            logger.info("%s advertised successfully ..." % (self.serviceName))
        except (Exception, BluetoothError, SystemExit, KeyboardInterrupt) as e:
            logger.error("Failed to advertise bluetooth services  ... ", exc_info=True)

    def acceptBluetoothConnection(self):
        try:
            self.clientSocket, clientInfo = self.serverSocket.accept()
            logger.info("Accepted bluetooth connection from %s", clientInfo)
        except (Exception, BluetoothError, SystemExit, KeyboardInterrupt) as e:
            logger.error("Failed to accept bluetooth connection ... ", exc_info=True)

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
            logger.info("Data received successfully over bluetooth connection")
            return data
        except (Exception, IOError, BluetoothError) as e:
            pass

    def deserializedData(self, _dataRecv):
        try:
            # print(_dataRecv)
            self.dataObj=pickle.loads(_dataRecv)
            logger.info("Serialized string converted successfully to object")
        except (Exception, pickle.UnpicklingError) as e:
            logger.error("Failed to de-serialized string ... ", exc_info=True)
            # print("Oh NO!!")

    def writeJsonFile(self):
        try:
            # Open a file for writing
            jsonFileObj = open(self.jsonFile,"w")
            logger.info("%s file successfully opened to %s" % (self.jsonFile, jsonFileObj))
            # Save the dictionary into this file
            # (the 'indent=4' is optional, but makes it more readable)
            json.dump(self.dataObj,jsonFileObj, indent=4)
            logger.info("Content dumped successfully to the %s file" %(self.jsonFile))
            # Close the file
            jsonFileObj.close()
            logger.info("%s file successfully closed" %(self.jsonFile))
        except (Exception, IOError) as e:
            logger.error("Failed to write json contents to the file ... ", exc_info=True)

    def closeBluetoothSocket(self):
        try:
            self.clientSocket.close()
            self.serverSocket.close()
            logger.info("Bluetooth sockets successfully closed ...")
        except (Exception, BluetoothError) as e:
            logger.error("Failed to close the bluetooth sockets ", exc_info=True)

    def start(self):
            # Create the server socket
            self.getBluetoothSocket()
            # get bluetooth connection to port # of the first available
            self.getBluetoothConnection()
            # advertising bluetooth services
            self.advertiseBluetoothService()
            # Accepting bluetooth connection
            self.acceptBluetoothConnection()

    def receive(self):
            # receive data
            dataRecv=self.recvData()
            print("received " + str(dataRecv))
            # de-serializing data
            self.deserializedData(dataRecv)
            # Writing json object to the file
            self.writeJsonFile()
            while True:
                try:
                    data = self.clientSocket.recv(1024)
                    if not data:
                        break
                    print("Received:", data.decode())
                    global current_location
                    current_location = data.decode()
                except (Exception, IOError) as e:
                    print("Bluetooth connection error:", e)
                    break

    def stop(self):
        # Disconnecting bluetooth sockets
        self.closeBluetoothSocket()


def blueSvrWork():
    startLogging()
    bleSvr = bleServer()
    bleSvr.start()
    bleSvr.receive()
    bleSvr.stop()


def current_milli_time():
    return round(time.time() * 1000)

if __name__ == '__main__':
    x = threading.Thread(target=blueSvrWork)
    x.start()
    # y = threading.Thread(target=test_BNO055Interface)
    # y.start()

    # bno = BNO_Interface.BNO055Interface()
    # bno.start()
    current_time = time.strftime("%M_%d_%H_%M", time.localtime())

    outfile = open("GPS" + str(current_time) + '.csv', 'w')
    try:
        # Print header for csv
        print("starting!")
        #print("time,", bno.get_csv_header(), ", latitude, longitude", file=outfile)
        delay_ms = 10 # 10 ms = 100hz
        time_old = current_milli_time()
        while(True):
            cur_time = current_milli_time()
            #if (cur_time - time_old >= delay_ms):
            if True:
                #time_old = cur_time
                t = time.localtime()
                current_time = time.strftime("%H:%M:%S", t)
                print(current_time, ",", cur_time%100000, ",", current_location, file=outfile)
                #time.sleep((delay_ms/2.0)*0.001) # Give CPU cycles to other thread
                cur_time2 = current_milli_time()
                # time.sleep((delay_ms / 3.0) * 0.001)
                time_to_delay = delay_ms * 0.001 - (cur_time2 - cur_time) * 0.001
                if (time_to_delay > 0):
                    time.sleep(time_to_delay)
                #print(current_time + " Most Recent BNO Data " + bno.get_csv_line() + " Most Recent GPS Data " + current_location)
                #print(current_time, ",", bno.get_csv_line(), ",", current_location, file=outfile)
    except KeyboardInterrupt:
        #print("Keyboard Interrupt debug 1")
        outfile.close()
        #x.join()
        #print("Keyboard Interrupt debug 2")
        #bno.stop()
