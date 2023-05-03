#!/usr/bin/env python3


import rospy
import time
import csv
from std_msgs.msg import String
from navigation.msg import bno
from navigation.msg import gps
from navigation.msg import reset

LOG_RATE = 50 # in hz

def copy_bno_data(msg_dst, msg_src):
    msg_dst.PosX = msg_src.PosX
    msg_dst.PosY = msg_src.PosY
    msg_dst.PosZ = msg_src.PosZ
    msg_dst.time_str = msg_src.time_str
    msg_dst.time_int = msg_src.time_int
    msg_dst.SystemCal = msg_src.SystemCal
    msg_dst.GyroCal = msg_src.GyroCal
    msg_dst.AccelCal = msg_src.AccelCal
    msg_dst.MagCal = msg_src.MagCal
    msg_dst.MagX = msg_src.MagX
    msg_dst.MagY = msg_src.MagY
    msg_dst.MagZ = msg_src.MagZ
    msg_dst.LinAccX = msg_src.LinAccX
    msg_dst.LinAccY = msg_src.LinAccY
    msg_dst.LinAccZ = msg_src.LinAccZ
    msg_dst.AbsAccX = msg_src.AbsAccX
    msg_dst.AbsAccY = msg_src.AbsAccY
    msg_dst.AbsAccZ = msg_src.AbsAccZ
    
def copy_gps_data(msg_dst, msg_src):
    msg_dst.lat = msg_src.lat
    msg_dst.lon = msg_src.lon
    
def f2s(x): # Convert float to string and set precision
    return '{:0.9f}'.format(x)
    
def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)
    
def gps_callback(gps_data):
    rospy.loginfo(rospy.get_caller_id() + 'From GPS: Lat: %f, Lon: %f', gps_data.lat, gps_data.lon)
    copy_gps_data(gps_msg_global,gps_data)
    
def bno_callback(bno_data):
    pass
    bno_callback.counter = bno_callback.counter + 1
    copy_bno_data(bno_msg_global, bno_data)
    if bno_callback.counter > 50:
        rospy.loginfo(rospy.get_caller_id() + 'From bno: %s, %f, posX: %f posY: %f', bno_data.time_str, bno_data.EulerX, bno_data.PosX, bno_data.PosY)
        bno_callback.counter = 0

bno_callback.counter = 0

def listener():
    global bno_msg_global
    bno_msg_global = bno()
    global gps_msg_global
    gps_msg_global = gps()
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber('chatter', String, callback)
    rospy.Subscriber('gps_data', gps, gps_callback)
    rospy.Subscriber('bno_data', bno, bno_callback)
    
    test_publisher = rospy.Publisher('reset_bno', reset, queue_size=10)
    
    rate = rospy.Rate(LOG_RATE)
    start_time = round(rospy.get_time() * 1000)
    current_time = time.strftime("%M_%d_%H_%M", time.localtime())
    outfile = open("/home/ubuntu/Desktop/navigation_log" + str(current_time) + '.csv', 'w')
    out_csv_writer = csv.writer(outfile, delimiter=',')
    out_csv_writer.writerow(["Time (ms)", " Time", "GPS Lat", "GPS Lon", "Pos x", "Pos y", "Pos z", "Sys Cal", "Gyro Cal", "Accel Cal", "Mag Cal", "Mag X", "Mag Y", "Mag Z", "Lin Acc X", "Lin Acc Y", "Lin Acc Z"]) # Write header
    while not rospy.is_shutdown():
        # Write to CSV
        out_csv_writer.writerow([str(round(rospy.get_time() * 1000) - start_time), bno_msg_global.time_str, f2s(gps_msg_global.lat), f2s(gps_msg_global.lon), f2s(bno_msg_global.PosX), f2s(bno_msg_global.PosY), f2s(bno_msg_global.PosZ), bno_msg_global.SystemCal, bno_msg_global.GyroCal, bno_msg_global.AccelCal, bno_msg_global.MagCal, f2s(bno_msg_global.MagX), f2s(bno_msg_global.MagY), f2s(bno_msg_global.MagZ), f2s(bno_msg_global.LinAccX), f2s(bno_msg_global.LinAccY), f2s(bno_msg_global.LinAccZ)]), 
        rate.sleep()
    	#test_msg = reset()
    	#test_msg.newLat = 0.0;
    	#test_msg.newLong = 0.0;
    	#test_publisher.publish(test_msg)
    
    outfile.close()

if __name__ == '__main__':
    listener()
