#!/usr/bin/env 

import rospy
import serial
from scipy.spatial.transform import Rotation as R
from imu_driver.srv import convert_to_quaternion, convert_to_quaternionResponse


def callback(request):
    #print(request)
    quaternion = R.from_euler('xyz', [request.roll, request.pitch, request.yaw], degrees=True).as_quat()
    qx = quaternion[0]
    qy = quaternion[1]
    qz = quaternion[2]
    qw = quaternion[3]
    return convert_to_quaternionResponse(qx, qy, qz, qw)


def start_service():
    rospy.init_node("convert_imu_service_server")
    service = rospy.Service("convert_to_quaternion", convert_to_quaternion, callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        start_service()
    except rospy.ROSInterruptException:
        pass
