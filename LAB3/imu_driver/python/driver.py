#!/usr/bin/env 
# -*- coding: utf-8 -*-
import rospy
import serial
from scipy.spatial.transform import Rotation as R
from imu_driver.srv import convert_to_quaternion
from imu_driver.msg import Vectornav


def readimu():
    rospy.init_node("imu_Publisher", anonymous=True)
    port = rospy.get_param('~/imu_Publisher/port')
    print(port)
    #port = '/dev/ttyUSB0'
    ser = serial.Serial(port, 115200, timeout = 1)
    ser.write(b'VNWRG,06,40\r\n')
    rospy.wait_for_service('convert_to_quaternion')
    pub = rospy.Publisher("/imu", Vectornav, queue_size= 10)
    msg = Vectornav()
    while not rospy.is_shutdown():
        x = ser.readline().decode("utf-8").strip()
        #print(x)
        line = str(x)
        if "VNYMR" in line:
            data = line.split(",")

            yaw = float(data[1])
            pitch = float(data[2])
            roll = float(data[3])
            magx = float(data[4])
            magy = float(data[5])
            magz = float(data[6])
            accx = float(data[7])
            accy = float(data[8])
            accz = float(data[9])
            gyrox = float(data[10])
            gyroy = float(data[11])
            gyroz = float(data[12][:10])
            
            try:
                convert_service = rospy.ServiceProxy('convert_to_quaternion', convert_to_quaternion)
                #print(roll, pitch, yaw)
                response = convert_service(roll, pitch, yaw)
                #print(response)
                qx = response.qx
                qy = response.qy
                qz = response.qz
                qw = response.qw
                #print(qx, qy, qz, qw)
            except rospy.ServiceException as e:
                print("service call failed")
            
            time = rospy.Time.now()
            msg.header.frame_id = "imu1_frame"
            msg.header.stamp.secs = time.to_sec
            msg.header.stamp.secs = time.to_nsec
            msg.raw = line
            msg.imu.orientation.x = qx
            msg.imu.orientation.y = qy
            msg.imu.orientation.z = qz
            msg.imu.orientation.w = qw

            msg.imu.linear_acceleration.x = accx
            msg.imu.linear_acceleration.y = accy
            msg.imu.linear_acceleration.z = accz

            msg.imu.angular_velocity.x = gyrox
            msg.imu.angular_velocity.y = gyroy
            msg.imu.angular_velocity.z = gyroz

            msg.mag_field.magnetic_field.x = magx
            msg.mag_field.magnetic_field.y = magy
            msg.mag_field.magnetic_field.z = magz
            print(msg.mag_field.magnetic_field.z, msg.imu.orientation.w)
            pub.publish()




      


if __name__ == '__main__':
    try:
        readimu()
    except rospy.ROSInterruptException:
        pass
