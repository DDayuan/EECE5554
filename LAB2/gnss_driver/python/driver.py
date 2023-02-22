#!/usr/bin/env 
# -*- coding: utf-8 -*-
import rospy
from curses.ascii import alt
import utm
from gnss_driver.msg import rtk_msg
import argparse
import serial
#print('reading')

def degreetodecimal(degree_type, degree):
    if degree_type == 'N' or degree_type == 'S':
        decimal = float(degree[:2]) + float(degree[2:]) / 60
    else:
        decimal = float(degree[:3]) + float(degree[3:]) / 60
    if degree_type == 'S' or degree_type == 'W':
        decimal = -decimal
    return decimal

def read_rtk():
    rospy.init_node("gps_Publisher", anonymous=True)
    param_names = rospy.get_param_names()
    port = rospy.get_param('/gps_Publisher/port')
    
	#baud = rospy.get_param('~gps_baudrate', 57600)
    ser = serial.Serial(port, 57600, timeout=1)
    #ser = serial.Serial('/dev/ttyACM0', 57600, timeout = 1)
    pub = rospy.Publisher("gnss", rtk_msg, queue_size= 10)
    msg = rtk_msgs()
    while not rospy.is_shutdown():
        line = str(ser.readline())
        if line != '':
            data = line.split(",")
            #print(data)
        if data[0] == 'b\'$GNGGA':
        #if line.startswith('$GNGGA'):
            print(data)
            #line = str(line.strip())
            #data = line.split(',')
            time = data[1].split('.')
            secs = (int(time[0][0]) * 10 + int(time[0][1])) * 3600 + (int(time[0][2]) * 10 + int(time[0][3])) * 60 + int(time[0][4]) * 10 + int(time[0][5])
            nsecs = int(time[1]) * 10000000
            latitude = degreetodecimal(data[3], data[2])
            longitude = degreetodecimal(data[5], data[4])
            utm_data = utm.from_latlon(latitude, longitude)
            print(data)

            msg.Header.seq = 0
            msg.Header.frame_id = "GPS1_Frame"
            msg.Header.stamp.secs = secs
            msg.Header.stamp.nsecs = nsecs
            msg.UTC = data[1]
            msg.Altitude = float(data[9])
            msg.HDOP = float(data[8])
            msg.Latitude = latitude
            msg.Longitude = longitude
            msg.UTM_easting = utm_data[0]
            msg.UTM_northing = utm_data[1]
            msg.Zone = utm_data[2]
            msg.Quality = int(data[6])
            msg.Letter = utm_data[3]
            print(msg.Quality)
            pub.publish(msg)



if __name__ == '__main__':
    try:
        read_rtk()
    except rospy.ROSInterruptException:
        pass
