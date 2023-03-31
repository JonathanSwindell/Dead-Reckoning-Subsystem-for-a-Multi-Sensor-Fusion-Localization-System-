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
            self.serialPort = serial.Serial(port=BNO_PORT, baudrate=115200)
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
        self.csv_lock = threading.Lock()

        self.euler = "e,e,e"
        self.linAccel = "la,la,la"
        self.gyro = "g,g,g"
        self.mag = "m,m,m"
        self.quat = "q,q,q,q"
        self.absAccel = "abs, abs, abs" # Absolute acceleration from BNO library. This is linear acceleration that has been adjusted by the rotation

    def run(self):
        while not self.stop_event.is_set():
            if self.serialPort is None:
                time.sleep(5)
                print("No serial port")
            elif self.serialPort.in_waiting > 0:
                # Read data out of the buffer until a carraige return / new line is found
                serialString = str(self.serialPort.readline().strip().decode('ascii'))
                # print(serialString)
                if "M:" in serialString:
                    with self.csv_lock:
                        self.mag = serialString[2:]
                elif "G:" in serialString:
                    with self.csv_lock:
                        self.gyro = serialString[2:]
                elif "Q:" in serialString:
                    with self.csv_lock:
                        self.quat = serialString[2:]
                elif "La:" in serialString:
                    with self.csv_lock:
                        self.linAccel = serialString[3:]
                elif "E:" in serialString:
                    with self.csv_lock:
                        self.euler = serialString[2:]
                elif "Aa:" in serialString:
                    with self.csv_lock:
                        self.absAccel = serialString[3:]
                # Write contents to output
                try:
                    with self.logging_lock:
                        if self.is_logging:
                            if "Location(X,Y,Z):" in serialString:
                                t = time.localtime()
                                current_time = time.strftime("%H:%M:%S", t)
                                print(current_time, ",", serialString[16:],file=self.outfile)
                            else:
                                pass
                                #print("FROM BNO:", serialString)
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

    def get_csv_line(self):
        with self.csv_lock:
            return self.mag + "," + self.gyro + "," + self.quat + "," + self.linAccel + "," + self.euler + "," + self.absAccel

    def get_csv_header(self):
        return "mag x, mag y, mag z, gyro x, gyro y, gyro z, quat x, quat y, quat z, quat w, linAcc x, linAcc y, linAcc z, Euler x, Euler y, Euler z, AbsAcc x, AbsAcc y, AbsAcc z"

    def reset_deadreck(self):
        print("Resetting the dead reckoning!")
        if self.serialPort is not None:
            self.serialPort.write("r\n".encode())  # Write reset command to serial

    def stop(self):
        self.stop_event.set()

    def __del__(self):
        self.stop()

def test_BNO055Interface():
    bno = BNO055Interface()
    bno.start()
    #bno.start_logging("test.csv")

    try:
        while True:
            time.sleep(1)
            print(bno.get_csv_line())
            #if input() == "r":
            #    bno.reset_deadreck()
    except KeyboardInterrupt:
        bno.stop_logging()
        bno.stop()
        bno.join()
    finally:
        bno.stop()
        bno.join()


if __name__ == "__main__":
    test_BNO055Interface()
