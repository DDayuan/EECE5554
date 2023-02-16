#!/usr/bin/env 
import rospy
import serial
import utm
import argparse
from gps_driver.msg import gps_msg

#latitude = ''
#longitude = ''
def readgps():
    #parser = argparse.ArgumentParser(description= "port path")
    #parser.add_argument('port', metavar='port', type = str, help = 'enter port path')
    #args = parser.parse_args()
    #port1 = args.port
    #print(port1)

    ser = serial.Serial('/dev/ttyUSB0', 4800, timeout = 1)
    #ser = serial.Serial(port1, 4800, timeout = 1)

    rospy.init_node("gps_Publisher", anonymous=True)
    pub = rospy.Publisher("/gps", gps_msg, queue_size= 10)
    msg = gps_msg()
    while not rospy.is_shutdown():
        x = ser.readline()
        print(x)
        line = str(x)
        if "GPGGA" in line:
            #print(line)
            data = line.split(",")

            raw_time = float(data[1])
            lat = float(data[2])
            longi = float(data[4])
            alti = float(data[9])

            utc_hrs = raw_time//10000
            utc_mint = (raw_time-(utc_hrs*10000))//100
            utc_sec = (raw_time - (utc_hrs*10000) - (utc_mint*100))
            utc_final_secs = (utc_hrs*3600 + utc_mint*60 + utc_sec)
            utc_final_nsecs = int((utc_final_secs * (10**7)) % (10**7))
            
            if data[3]=='S':
                lat = -lat

            if data[5]=='W':
                longi = -longi

            lat_degree = lat // 100
            longi_degree = longi // 100
            lat_min = lat - (lat_degree * 100)
            longi_min = longi - (longi_degree * 100)
            new_lat = float(lat_degree + lat_min)
            new_longi = float(longi_degree + longi_min)

            utm_data = utm.from_latlon(new_lat, new_longi)
            #msg.Header.stamp.secs = int(data[1])
            msg.Header.stamp.secs = int(utc_final_secs)
            msg.Header.stamp.nsecs = int(utc_final_nsecs)
            msg.Header.frame_id ="GPS1_Frame"
            msg.Latitude = new_lat
            msg.Longitude = new_longi
            msg.Altitude = alti
            msg.HDOP = float(data[8])
            msg.UTM_easting = utm_data[0]
            msg.UTM_northing = utm_data[1]
            msg.UTC = str(str(utc_hrs) + ":" + str(utc_mint))
            msg.Zone = utm_data[2]
            msg.Letter = utm_data[3]
            pub.publish(msg)


if __name__ == '__main__':
    try:
        readgps()
    except rospy.ROSInterruptException:
        pass

