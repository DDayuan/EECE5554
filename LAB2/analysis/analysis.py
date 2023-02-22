#!/usr/bin/env
import numpy as np
import matplotlib.pyplot as plt
import bagpy
import seaborn as sea
import pandas as pd

def plot_easting_vs_northing(easting_data, northing_data, pre_string):
    plt.title(pre_string + 'northing vs easting')
    plt.scatter(northing_data, easting_data, edgecolor='tab:blue', facecolors='none', label='collection data')
    plt.xlabel('UTM Northing')
    plt.ylabel('UTM Easting')
    plt.legend()
    plt.grid(linestyle=':')
    plt.show()

def plot_altitude_vs_time(altitude_data, altitude_mean, pre_string):
    plt.plot(altitude_data, label='collection data')
    plt.xlabel('Data Point')
    plt.ylabel('Altitude (in meters)')
    plt.title(pre_string + 'Altitude vs Time')
    plt.grid(linestyle=':')
    plt.show()

    
def plot_static(easting_mean, northing_mean, easting_data, northing_data, pre_string):
    static_error = np.sqrt(np.square(easting_data - easting_mean) + np.square(northing_data - northing_mean))
    for error in static_error:
        if abs(error) < 1:
            error = 0
        if error < 0:
            error = -error
    plt.hist(static_error, bin=100)
    plt.title(pre_string + 'Error of UTM for Static Data' )
    plt.xlabel('Distance From ground truth (in meters)')
    utm_std_dev = np.std((northing_data - northing_mean, easting_data - easting_mean))
    print('UTM Standard Deviation: ', utm_std_dev)
    plt.ylabel('Number of Data')
    plt.grid(linestyle=':')
    plt.show

def plot_moving(easting_data, northing_data, pre_string):
    if pre_string == 'open_area':
        print('open_area')
        p1=[328315, 4689587]
        p2=[328299, 4689570]
        p3=[328309, 4689560]
        p4=[328327, 4689578]
        p5 = p1
        a1=0
        a2=2247
        b1=a2
        b2=3253
        c1=b2
        c2=4850
        d1=c2

        

    elif pre_string == 'occlude':
        print('occlude')
        p1=[328087, 4689325]
        p2=[328107, 4689336]
        p3=[328101, 4689347]
        p4=[328084, 4689337]
        p5 = p1    
        a1=398
        a2=1651
        b1=a2
        b2=2203
        c1=b2
        c2=3005
        d1=2999

    plt.figure()
    k1,bias1 = np.polyfit((p1[0], p2[0]),(p1[1], p2[1]),1)
    error1 = k1*easting_data[a1:a2]+bias1-northing_data[a1:a2]
    k2,bias2 = np.polyfit((p2[0], p3[0]),(p2[1], p3[1]),1)
    error2 = k2*easting_data[b1:b2]+bias2-northing_data[b1:b2]
    k3,bias3 = np.polyfit((p3[0], p4[0]),(p3[1], p4[1]),1)
    error3 = k3*easting_data[c1:c2]+bias3-northing_data[c1:c2]
    k4, bias4 = np.polyfit((p4[0], p4[0]),(p5[1], p5[1]),1)
    error4 = k4*easting_data[d1:]+bias4-northing_data[d1:]
    error_all=np.array([])
    error_all=np.append(error_all, error1)
    error_all=np.append(error_all, error2)
    error_all=np.append(error_all, error3)
    error_all=np.append(error_all, error4)

    plt.hist(error_all,bins=20)
    plt.title(pre_string + 'error_hist_deviation')
    plt.show()





if __name__ == '__main__':

    data_path = '../data/occ_walking.yaml'

    if 'open' in data_path:
        pre_string = 'open_area'
    else:
        pre_string = 'occlude'
    
    static = False
    if 'station' in data_path:
        static = True

    easting_data = []
    northing_data = []
    altitude_data = []
    longitude_data = []
    latitude_data = []

    with open(data_path) as f:
        lines = f.readlines()


        #print(len(lines))
        i = 0

        for line in lines:
            i += 1
            if 'altitude' in line:
                altitude = float(line[10:])
                altitude_data.append(altitude)

            if 'latitude' in line:
                latitude = float(line[10:])
                latitude_data.append(latitude)
        
            if 'longitude' in line:
                longitude = float(line[11:])
                longitude_data.append(longitude)
        
            if 'utm_northing' in line:
                northing = float(line[13:])
                northing_data.append(northing)
        
            if 'utm_easting' in line:
                easting = float(line[12:])
                print(easting)
                easting_data.append(easting)
    f.close()
    
    easting_mean = np.average(easting_data)
    northing_mean = np.average(northing_data)
    altitude_mean = np.average(altitude_data)
    print(easting_data)

    plot_easting_vs_northing(easting_data, northing_data, pre_string)
    plot_altitude_vs_time(altitude_data, altitude_mean, pre_string)

    if static:
        plot_static(easting_mean, northing_mean, easting_data, northing_data, pre_string)
    else:
        plot_moving(easting_data, northing_data, pre_string)
