import bagpy
import math
import csv
import time
import statistics
from bagpy import bagreader
import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns
import pandas as pd

bag = bagreader('/home/dayuan/eece5554/lab4/2023-03-29-13-32-55.bag')
data = bag.message_by_topic('/imu')
#data = bag.message_by_topic('/gps')
readings = pd.read_csv(data)
plt.scatter(readings['mag_field.magnetic_field.x'], readings['mag_field.magnetic_field.y'], marker='.', label='Raw/Uncalibrated Data')
#print(readings['UTM_easting'][1:50])
plt.show()
