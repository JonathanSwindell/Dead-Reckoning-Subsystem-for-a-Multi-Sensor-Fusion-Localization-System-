import time
import serial
import threading

port = "COM4"

class KeyboardThread(threading.Thread):

    def __init__(self, serialPort, input_cbk = None, name='keyboard-input-thread'):
        self.input_cbk = input_cbk
        self.serialPort = serialPort
        super(KeyboardThread, self).__init__(name=name)
        self.start()

    def run(self):
        while True:
            self.input_cbk(self.serialPort, input()) #waits to get input + Return


def my_callback(serialPort, inp):
    print("sending ", inp, " to the device.")
    serialPort.write(inp)

def main():
    serialPort = serial.Serial(port=port, baudrate=9600)
    serialString = ""  # Used to hold data coming over UART

    kthread = KeyboardThread(serialPort=serialPort, input_cbk=my_callback)

    while 1:
        # Wait until there is data waiting in the serial buffer
        if serialPort.in_waiting > 0:

            # Read data out of the buffer until a carraige return / new line is found
            serialString = serialPort.readline()

            # Print the contents of the serial data
            try:
                print(serialString.decode("Ascii"))
            except:
                pass

if __name__ == "__main__":
    main()