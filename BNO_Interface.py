import serial
import threading
import time

BNO_PORT = "COM4"


class BNO055Interface(threading.Thread):
    __instance = None

    def __new__(cls):
        if cls.__instance is None:
            cls.__instance = super(BNO055Interface, cls).__new__(cls)
        return cls.__instance

    def __init__(self):
        super(BNO055Interface, self).__init__()
        try:
            self.serialPort = serial.Serial(port=BNO_PORT, baudrate=9600)
        except:
            print("Error in BNO055Interface. Unable to open serialPort")
            self.serialPort = None
        self.xpos = 0.0
        self.ypos = 0.0
        self.zpos = 0.0
        self.is_logging = False
        self.outfile = None
        self.logging_lock = threading.Lock()
        self.stop_event = threading.Event()

    def run(self):
        while not self.stop_event.is_set():
            if self.serialPort is None:
                time.sleep(5)
                print("No serial port")
            elif self.serialPort.in_waiting > 0:
                # Read data out of the buffer until a carraige return / new line is found
                serialString = self.serialPort.readline()

                # Write contents to output
                try:
                    with self.logging_lock:
                        if self.is_logging:
                            if "Location(X,Y,Z):" in serialString:
                                self.outfile.write(serialString[15:])
                            else:
                                print("FROM BNO:", serialString)
                        else:
                            pass  # Maybe print here? idk

                except:
                    pass

    def start_logging(self, filename):
        with self.logging_lock:
            print("Starting Logging!")
            self.outfile = open(filename, 'w')
            self.is_logging = True

    def stop_logging(self):
        with self.logging_lock:
            print("Stopping Logging!")
            if self.outfile is not None:
                self.outfile.close()

    def get_position(self):
        return self.xpos, self.ypos, self.zpos  # TODO: update xpos,ypos,zpos in run()

    def reset_deadreck(self):
        print("Resetting the dead reckoning!")
        if self.serialPort is not None:
            self.serialPort.write("r\n")  # Write reset command to serial

    def stop(self):
        self.stop_event.set()

def test_BNO055Interface():
    bno = BNO055Interface()
    bno.start()
    try:
        while True:
            if input() == "r":
                bno.reset_deadreck()
    except KeyboardInterrupt:
        bno.stop()
        bno.join()
    finally:
        bno.stop()
        bno.join()


if __name__ == "__main__":
    test_BNO055Interface()
