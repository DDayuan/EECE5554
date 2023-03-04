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

#bag = bagreader('/../Data/sta_five_min.bag')

bag = bagreader('/home/dayuan/eece5554/lab3/src/Data/sta_five_min.bag')
data = bag.message_by_topic('/imu')
readings = pd.read_csv(data)

#calculate mean gyro x y z
mean_gyrox = readings['imu.angular_velocity.x'].mean()
mean_gyroy = readings['imu.angular_velocity.y'].mean()
mean_gyroz = readings['imu.angular_velocity.z'].mean()
print('gyro_x mean =', mean_gyrox, 'median =', readings['imu.angular_velocity.x'].median())
print('gyro_y mean = ', mean_gyroy, 'median =', readings['imu.angular_velocity.y'].median())
print('gyro_z mean =', mean_gyroz, 'median =', readings['imu.angular_velocity.x'].median())

#calculate mean accle x y z
mean_accx = readings['imu.linear_acceleration.x'].mean()
mean_accy = readings['imu.linear_acceleration.y'].mean()
mean_accz = readings['imu.linear_acceleration.z'].mean()
stdx = statistics.stdev(readings['imu.linear_acceleration.x'])
print('acceleration_x mean =', mean_accx, 'median =', readings['imu.linear_acceleration.x'].median(), 'standard deviation =', stdx)
print('acceleration_y mean = ', mean_accy,'median =', readings['imu.linear_acceleration.y'].median())
print('acceleration_z mean =', mean_accz,'median =', readings['imu.linear_acceleration.z'].median())

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

#calculate mean orientation x y z
mean_roll = roll.mean()
mean_pitch = pitch.mean()
mean_yaw = yaw.mean()
print('mean orientation x =', mean_roll, 'median =', statistics.median(roll))
print('mean orientation y =', mean_pitch, 'median =', statistics.median(pitch))
print('mean orientation z =', mean_yaw, 'median =', statistics.median(yaw))

#plot gyro x y z
plt.plot(readings['imu.angular_velocity.x'], label = "angular x")
plt.plot(readings['imu.angular_velocity.y'], label = "angular y")
plt.plot(readings['imu.angular_velocity.z'], label = "angular z")
plt.xlabel('time')
plt.ylabel('rad/s')
plt.title('gyro xyz')
plt.legend()
plt.show()

#plot accel x y z
plt.plot(readings['imu.linear_acceleration.x'], label = "accel x")
plt.plot(readings['imu.linear_acceleration.y'], label = "accel y")
plt.plot(readings['imu.linear_acceleration.z'], label = "accel z")
plt.xlabel('time')
plt.ylabel('m/s^2')
plt.title('acceleration xyz')
plt.legend()
plt.show()

#plot orientation x y z
plt.plot(roll, label = "orientation x")
plt.plot(pitch, label = "orientation y")
plt.plot(yaw, label = "orientations z")
plt.xlabel('time')
plt.ylabel('Gaussian')
plt.title('orientation xyz')
plt.legend()
plt.show()

#plot histogram
fig, ax = plt.subplots(figsize =(10, 7))
ax.hist(readings['imu.linear_acceleration.x'])
plt.ylabel('occuring_times')
plt.xlabel('accel x(m/s^2)')
plt.show()
