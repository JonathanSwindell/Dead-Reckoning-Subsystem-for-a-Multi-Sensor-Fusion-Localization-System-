#!/usr/bin/env python3
# license removed for brevity
import rospy

import serial
import time

from std_msgs.msg import String
from navigation.msg import bno
from navigation.msg import gps

BNO_MESSAGE_RATE = 100 # In hz
BNO_PORT = "/dev/ttyS4"

def bno_interface():
    bno_publisher = rospy.Publisher('bno_data', bno, queue_size=10)
    rospy.init_node('BNO_Interface', anonymous=True)
    rate = rospy.Rate(BNO_MESSAGE_RATE)
    serialPort = None
    try:
        serialPort = serial.Serial(port=BNO_PORT, baudrate=115200)
    except:
        print("Error in BNO055Interface. Unable to open serialPort")
    while not rospy.is_shutdown():
        # Collect BNO Data, and then publish it.
        bno_msg = bno() # new BNO message
        msg_count = 0
        if serialPort is None:
                time.sleep(5)
                print("No serial port")
        else:
            while msg_count < 8:
                # Read data out of the buffer until a carraige return / new line is found
                while (serialPort.in_waiting <= 0):
                    pass
                
                serialString = str(serialPort.readline().strip().decode('ascii'))
                if "C:" in serialString:
                    #rospy.loginfo("C")
                    calibration = serialString[2:].split(',')
                    bno_msg.SystemCal = int(calibration[0])
                    bno_msg.GyroCal = int(calibration[1])
                    bno_msg.AccelCal = int(calibration[2])
                    bno_msg.MagCal = int(calibration[3])
                elif "Lo:" in serialString:
                    #rospy.loginfo("Lo")
                    position = serialString[3:].split(',')
                    bno_msg.PosX = float(position[0])
                    bno_msg.PosY = float(position[1])
                    bno_msg.PosZ = float(position[2])
                elif "M:" in serialString:
                    #rospy.loginfo("M")
                    mag = serialString[2:].split(',')
                    bno_msg.MagX = float(mag[0])
                    bno_msg.MagY = float(mag[1])
                    bno_msg.MagZ = float(mag[2])
                elif "G:" in serialString:
                    #rospy.loginfo("G")
                    gyro = serialString[2:].split(',')
                    bno_msg.GyroX = float(gyro[0])
                    bno_msg.GyroY = float(gyro[1])
                    bno_msg.GyroZ = float(gyro[2])
                elif "Q:" in serialString:
                    #rospy.loginfo("Q")
                    quat = serialString[2:].split(',')
                    bno_msg.QuatX = float(quat[0])
                    bno_msg.QuatY = float(quat[1])
                    bno_msg.QuatZ = float(quat[2])
                    bno_msg.QuatW = float(quat[3])
                elif "La:" in serialString:
                    #rospy.loginfo("La")
                    linAccel = serialString[3:].split(',')
                    bno_msg.LinAccX = float(linAccel[0])
                    bno_msg.LinAccY = float(linAccel[1])
                    bno_msg.LinAccZ = float(linAccel[2])
                elif "E:" in serialString:
                    #rospy.loginfo("E")
                    euler = serialString[2:].split(',')
                    bno_msg.EulerX = float(euler[0])
                    bno_msg.EulerY = float(euler[1])
                    bno_msg.EulerZ = float(euler[2])
                elif "Aa:" in serialString:
                    #rospy.loginfo("Aa")
                    absAccel = serialString[3:].split(',')
                    bno_msg.AbsAccX = float(absAccel[0])
                    bno_msg.AbsAccY = float(absAccel[1])
                    bno_msg.AbsAccZ = float(absAccel[2])
                msg_count += 1
            msg_count = 0
            # Publish the message
            bno_msg.time_str = current_time = time.strftime("%M_%d_%H_%M", time.localtime())
            bno_msg.time_int = round(time.time() * 1000) # In ms
            bno_publisher.publish(bno_msg)
            # rospy.loginfo("sending message")
            #rate.sleep()    # Might have an issue here getting all of the messages. Given that
                            # The inner while loop waits for messages, having this node also
                            # sleep might be unnesarry


if __name__ == '__main__':
    try:
        bno_interface()
    except rospy.ROSInterruptException:
        pass