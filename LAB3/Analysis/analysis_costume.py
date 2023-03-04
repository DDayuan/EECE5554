import bagpy
import statistics
from bagpy import bagreader
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import pandas as pd
import csv
import math
from scipy.spatial.transform import Rotation as R
from tf.transformations import euler_from_quaternion, quaternion_from_euler

bag = bagreader('/home/dayuan/eece5554/lab3/src/Data/recordteam.bag')
data = bag.message_by_topic('/imu')
readings = pd.read_csv(data)

#quaternian back to euler
qx = readings['imu.orientation.x']
qy = readings['imu.orientation.y']
qz = readings['imu.orientation.z']
qw = readings['imu.orientation.w']
orientation_list = [qx, qy, qz, qw]
t0 = 2.0 *(qw * qx + qy * qz)
t1 = 1.0 - 2.0 *(qx * qx + qy * qy)
roll = np.degrees(np.arctan2(t0, t1))

t2 = +2.0 * (qw * qx - qy * qz)
t2 = np.where(t2>+1.0, +1.0,t2)
t2 = np.where(t2<-1.0, -1.0,t2)
pitch = np.degrees(np.arcsin(t2))

t3 = +2.0 * (qw * qz + qx * qy)
t4 = +1.0 - 2.0 * (qy * qy+ qz * qz)
yaw = np.degrees(np.arctan2(t3, t4))


#plot first move
# gyro x y z
plt.plot(readings['imu.angular_velocity.x'][500 : 1000], label = "angular x")
plt.plot(readings['imu.angular_velocity.y'][500: 1000], label = "angular y")
plt.plot(readings['imu.angular_velocity.z'][500 : 1000], label = "angular z")
plt.legend()
plt.title('gyro xyz for first move')
plt.show()

#plot accel x y z
plt.plot(readings['imu.linear_acceleration.x'][200 : 1000], label = "accel x")
plt.plot(readings['imu.linear_acceleration.y'][200 : 1000], label = "accel y")
plt.plot(readings['imu.linear_acceleration.z'][200 : 1000], label = "accel z")
plt.legend()
plt.title('acceleration xyz for first move')
plt.show()

""" #plot orientation x y z
plt.plot(roll[200 : 1000], label = "orientation x")
plt.plot(pitch[200 : 1000], label = "orientation y")
plt.plot(yaw[200 : 1000], label = "orientations z")
plt.legend()
plt.title('orientation for first move')
plt.show()  """

#plot second move
# gyro x y z
plt.plot(readings['imu.angular_velocity.x'][8500 : 9000], label = "angular x")
plt.plot(readings['imu.angular_velocity.y'][8500: 9000], label = "angular y")
plt.plot(readings['imu.angular_velocity.z'][8500 : 9000], label = "angular z")
plt.legend()
plt.title('gyro xyz for second move')
plt.show()

#plot accel x y z
plt.plot(readings['imu.linear_acceleration.x'][8500 : 9000], label = "accel x")
plt.plot(readings['imu.linear_acceleration.y'][8500 : 9000], label = "accel y")
plt.plot(readings['imu.linear_acceleration.z'][8500 : 9000], label = "accel z")
plt.legend()
plt.title('acceleration xyz for second move')
plt.show()

""" #plot orientation x y z
plt.plot(roll[8500 : 9000], label = "orientation x")
plt.plot(pitch[8500 : 9000], label = "orientation y")
plt.plot(yaw[8500 : 9000], label = "orientations z")
plt.legend()
plt.title('orientation for second move')
plt.show()

print(pitch[2000])
print(pitch[5000])
print(pitch[8000])
print(pitch[10000])
 """


#plot third move
""" #plot orientation x y z
plt.plot(roll[5900 : 6300], label = "orientation x")
plt.plot(pitch[5900 : 6300], label = "orientation y")
plt.plot(yaw[5900 : 6300], label = "orientations z")
plt.legend()
plt.title('orientation for third move')
plt.show()
 """
plt.plot(readings['imu.angular_velocity.x'][11000: 12500], label = "angular x")
plt.plot(readings['imu.angular_velocity.y'][11000: 12500], label = "angular y")
plt.plot(readings['imu.angular_velocity.z'][11000 : 12500], label = "angular z")
plt.legend()
plt.title('gyro xyz for third move')
plt.show()

#plot accel x y z
plt.plot(readings['imu.linear_acceleration.x'][11000: 12500], label = "accel x")
plt.plot(readings['imu.linear_acceleration.y'][11000: 12500], label = "accel y")
plt.plot(readings['imu.linear_acceleration.z'][11000: 12500], label = "accel z")
plt.legend()
plt.title('acceleration xyz for third move')
plt.show()
