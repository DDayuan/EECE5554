#!/usr/bin
import numpy as np
import matplotlib.pyplot as plt
import bagpy
from bagpy import bagreader
import seaborn as sea
import pandas as pd



bag_file = bagpy.bagreader('/home/dayuan/catkin_ws/src/gps_driver/Data/gps_straight_line.bag')



data = bag_file.message_by_topic('/gps')
readings = pd.read_csv(data)
#readings[['Heaser.stamp.secs', 'Altitude']].plot()
plt.rcParams.update({'font.size': 40})

fig, ax = bagpy.create_fig(1)
ax[0].scatter(x = 'Altitude', y = 'Header.stamp.secs', data = readings, s= 50, label = 'Altitude VS Time')
for axis in ax:
    axis.legend()
    axis.set_xlabel('Altitude', fontsize=40)
    axis.set_ylabel('Time', fontsize=40)
plt.show()
readings['UTM_easting'] = readings['UTM_easting'] - readings['UTM_easting'].min()
readings['UTM_northing'] = readings['UTM_northing'] - readings['UTM_northing'].min()

readings['straight_line_error'] = abs(readings['UTM_northing'] - (-2.71768 * readings['UTM_easting'] + 7168.06))
#error.append(2.71768 * readings['UTM_easting'] - 7168.06)
fig, ax = plt.subplots(figsize =(10, 7))
ax.hist(readings['straight_line_error'])
plt.xlabel('error_distances(in cm)')
plt.ylabel('occuring_time')

plt.show()
