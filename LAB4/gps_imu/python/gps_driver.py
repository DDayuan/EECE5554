#!/usr/bin/env 
import rospy
import serial
import utm
import argparse
from gps_imu.msg import gps_msg


def readgps():
    rospy.init_node("gps_Publisher", anonymous=True)
    port = rospy.get_param('~/gps_Publisher/gps_port')

    #ser = serial.Serial('/dev/ttyUSB1', 4800, timeout = 1)
    ser = serial.Serial(port, 4800, timeout = 1)

    #rospy.init_node("gps_Publisher", anonymous=True)
    pub = rospy.Publisher("/gps", gps_msg, queue_size= 10)
    msg = gps_msg()


    while not rospy.is_shutdown():
        x = ser.readline().decode("utf-8").strip()
        line = str(x)
        if "GPGGA" in line:
            try:
                data = line.split(",")
                lat = data[2]
                longi = data[4]
                alti = float(data[9])

                time = data[1].split('.')
                secs = (int(time[0][0]) * 10 + int(time[0][1])) * 3600 + \
			   	       (int(time[0][2]) * 10 + int(time[0][3])) * 60   + \
					    int(time[0][4]) * 10 + int(time[0][5])
						    	
                nsecs = int(time[1]) * pow(10,6)
            
            
                new_lat = float(lat[:2]) + float(lat[2:]) / 60
                new_longi = float(longi[:2]) + float(longi[2:]) / 60
            
                if data[3]=='S':
                    lat = -new_lat

                if data[5]=='W':
                    longi = -new_longi
                utm_data = utm.from_latlon(new_lat, new_longi)
            


                msg.Header.stamp.secs = secs
                msg.Header.stamp.nsecs = nsecs
                msg.Header.frame_id = "GPS1_FRAME"
                msg.Latitude = new_lat
                msg.Longitude = new_longi
                msg.Altitude = alti
                msg.HDOP = float(data[8])
                msg.UTM_easting = utm_data[0]
                msg.UTM_northing = utm_data[1]
                #msg.UTC = str(str(utc_hrs) + ":" + str(utc_mint))
                msg.Zone = utm_data[2]
                msg.Letter = utm_data[3]
            except:
                rospy.logwarn("Data exception: " + line)
                continue

            pub.publish(msg)
            print("gps HDOP is:", msg.HDOP)


if __name__ == '__main__':
    try:
        readgps()
    except rospy.ROSInterruptException:
        pass

