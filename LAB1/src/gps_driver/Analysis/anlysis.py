#!/usr/bin
import numpy as np
import matplotlib.pyplot as plt
import bagpy
from bagpy import bagreader
import seaborn as sea
import pandas as pd


bag_file = bagpy.bagreader('/home/dayuan/catkin_ws/src/gps_driver/Data/gps_open_area.bag')
#bag_file = bagpy.bagreader('/home/dayuan/catkin_ws/src/gps_driver/Data/gps_straight_line.bag')
bag_file = bagpy.bagreader('/home/dayuan/catkin_ws/src/gps_driver/Data/gps_close_building_area.bag')

data = bag_file.message_by_topic('/gps')
readings = pd.read_csv(data)


readings['UTM_easting'] = readings['UTM_easting'] - readings['UTM_easting'].min()
readings['UTM_northing'] = readings['UTM_northing'] - readings['UTM_northing'].min()

#readings['northing_offset'] = 
easting_mean = readings['UTM_easting'].mean()
northing_mean = readings['UTM_northing'].mean()
readings['x_square'] = (readings['UTM_northing'] - northing_mean) * (readings['UTM_northing'] - northing_mean)
readings['y_square'] = (readings['UTM_easting'] - easting_mean) * (readings['UTM_easting'] - easting_mean)
readings['error_distance'] = pow((readings['x_square'] + readings['y_square']), 0.5)
#readings['error_distance'] = readings['x_square'] + readings['y_square']
#print(readings['UTM_easting'])
#print(readings['error_distance'])
mean_error = readings['error_distance'].mean()
median_dis = readings['error_distance'].median()
readings['distance_to_mean'] = readings['error_distance'] - mean_error
#print(readings['distance_to_mean'])

#sum_error = sum(readings['error_distance'])
print(mean_error)
print(median_dis)

fig, ax = plt.subplots(figsize =(10, 7))
ax.hist(readings['error_distance'])
plt.ylabel('occuring_times')
plt.xlabel('error distances(in cm)')
plt.show()

#readings[['UTM_easting','UTM_northing']].plot()
fig, ax = bagpy.create_fig(1)
ax[0].scatter(x = 'UTM_easting', y = 'UTM_northing', data = readings, s= 50, label = 'UTM_easting VS UTM_northing')
for axis in ax:
    axis.legend()
    axis.set_xlabel('UTM_easting(cm)', fontsize=40)
    axis.set_ylabel('UTM_northing(cm)', fontsize=40)
plt.show()

fig, ax = bagpy.create_fig(1)
ax[0].scatter(x = 'Altitude', y = 'Header.stamp.secs', data = readings, s= 50, label = 'Altitude VS Time')
for axis in ax:
    axis.legend()
    axis.set_xlabel('Altitude(cm)', fontsize=40)
    axis.set_ylabel('Time(s)', fontsize=40)
plt.show()