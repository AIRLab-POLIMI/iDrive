#!/usr/bin/env python

import rospy
import time
from datetime import datetime
import signal
import numpy as np
from matplotlib import dates
import pylab as plt
import threading as thrd

#messages
from idrive_data_logger.msg import Bio_sensor

#global
plotter_seconds = 0
BVP = np.zeros(1)
skin = np.zeros(1)
ECG = np.zeros(1)
temp = np.zeros(1)
resp = np.zeros(1)
SEMG = np.zeros(1)
BVP_t = np.zeros(1)
skin_t = np.zeros(1)
ECG_t = np.zeros(1)
temp_t = np.zeros(1)
resp_t = np.zeros(1)
SEMG_t = np.zeros(1)

def plot_simulation(BVP_temp, skin_temp, ECG_temp, temp_temp, resp_temp, SEMG_temp, BVP_t, skin_t, ECG_t, temp_t, resp_t, SEMG_t):
    hfmt = dates.DateFormatter("%H:%M:%S.%f")
    fig1 = plt.figure(1,figsize=(16,10))
    ax1 = plt.subplot(321)
    ax1.clear()
    ax1.plot_date(skin_t, skin_temp, 'r')
    ax1.set_ylabel('skin conductance')
    ax1.xaxis.set_major_formatter(hfmt)
    for tick in ax1.xaxis.get_major_ticks():
        tick.label.set_fontsize(8)
    ax1.grid('on')
    ax2 = plt.subplot(323)
    ax2.clear()
    ax2.plot_date(BVP_t, BVP_temp, 'r')
    ax2.set_ylabel('BVP')
    ax2.xaxis.set_major_formatter(hfmt)
    for tick in ax2.xaxis.get_major_ticks():
        tick.label.set_fontsize(8)
    ax2.grid('on')
    ax3 = plt.subplot(325)
    ax3.clear()
    ax3.plot_date(ECG_t, ECG_temp, 'r')
    ax3.set_ylabel('ECG')
    ax3.xaxis.set_major_formatter(hfmt)
    for tick in ax3.xaxis.get_major_ticks():
        tick.label.set_fontsize(8)
    ax3.grid('on')
    ax4 = plt.subplot(322)
    ax4.clear()
    ax4.plot_date(temp_t, temp_temp, 'r')
    ax4.set_ylabel('temperature')
    ax4.xaxis.set_major_formatter(hfmt)
    for tick in ax4.xaxis.get_major_ticks():
        tick.label.set_fontsize(8)
    ax4.set_ylim(30,40)
    ax4.grid('on')
    ax5 = plt.subplot(324)
    ax5.clear()
    ax5.plot_date(resp_t, resp_temp, 'r')
    ax5.set_ylabel('respiration')
    ax5.xaxis.set_major_formatter(hfmt)
    for tick in ax5.xaxis.get_major_ticks():
        tick.label.set_fontsize(8)
    ax5.grid('on')
    ax6 = plt.subplot(326)
    ax6.clear()
    ax6.plot_date(SEMG_t, SEMG_temp, 'r')
    ax6.set_ylabel('SEMG')
    ax6.xaxis.set_major_formatter(hfmt)
    for tick in ax6.xaxis.get_major_ticks():
        tick.label.set_fontsize(8)
    ax6.grid('on')
    plt.draw()
    plt.show(block=False)

def time(time, lenght):
    d = 125000000 // lenght
    conv = np.zeros(lenght)
    for i in range(lenght):
	nsec = d * (lenght - i)
        temp = time - rospy.Duration(0, nsec)
        conv[i] = temp.to_sec()
    dts = map(datetime.utcfromtimestamp, conv)
    fds = dates.date2num(dts)
    return fds

def real_update(data):
    global BVP, skin, ECG, temp, resp, SEMG, BVP_t, skin_t, ECG_t, temp_t, resp_t, SEMG_t
    data_lenght = len(data.data)
    if (data.sensor_type == 'BVP'):
        BVP = np.concatenate((BVP[data_lenght:],data.data))
        BVP_t = np.concatenate((BVP_t[data_lenght:], time(data.header.stamp, data_lenght)))		
    elif (data.sensor_type == 'Skin conductance'):
        skin = np.concatenate((skin[data_lenght:],data.data))
	skin_t = np.concatenate((skin_t[data_lenght:], time(data.header.stamp, data_lenght)))
    elif (data.sensor_type == 'ECG'):
        ECG = np.concatenate((ECG[data_lenght:],data.data))
	ECG_t = np.concatenate((ECG_t[data_lenght:], time(data.header.stamp, data_lenght)))
    elif (data.sensor_type == 'Temperature'):
        temp = np.concatenate((temp[data_lenght:],data.data))
	temp_t = np.concatenate((temp_t[data_lenght:], time(data.header.stamp, data_lenght)))
    elif (data.sensor_type == 'Respiration'):
        resp = np.concatenate((resp[data_lenght:],data.data))
	resp_t = np.concatenate((resp_t[data_lenght:], time(data.header.stamp, data_lenght)))
    elif (data.sensor_type == 'SEMG'):
        SEMG = np.concatenate((SEMG[data_lenght:],data.data))
	SEMG_t = np.concatenate((SEMG_t[data_lenght:], time(data.header.stamp, data_lenght)))

def plotter():
    global BVP, skin, ECG, temp, resp, SEMG, BVP_t, skin_t, ECG_t, temp_t, resp_t, SEMG_t

    rospy.init_node('plotter', anonymous=True)
    plotter_seconds = rospy.get_param('~seconds_window')

    BVP = np.zeros(256 * plotter_seconds)
    skin = np.zeros(256 * plotter_seconds)
    ECG = np.zeros(2048 * plotter_seconds)
    temp = np.zeros(256 * plotter_seconds)
    resp = np.zeros(256 * plotter_seconds)
    SEMG = np.zeros(2048 * plotter_seconds)
    dts256 = map(datetime.utcfromtimestamp, np.zeros(256 * plotter_seconds))
    dts2048 = map(datetime.utcfromtimestamp, np.zeros(2048 * plotter_seconds))
    BVP_t = dates.date2num(dts256)
    skin_t = dates.date2num(dts256)
    ECG_t = dates.date2num(dts2048)
    temp_t = dates.date2num(dts256)
    resp_t = dates.date2num(dts256)
    SEMG_t = dates.date2num(dts2048)

    rospy.Subscriber("procomp_sensor", Bio_sensor,real_update)

    delta_t = rospy.get_param('~delta_time')
    rate = rospy.Rate(1/delta_t)
    
    while not rospy.is_shutdown():
        # Plots	
        plot_simulation(BVP, skin, ECG, temp, resp, SEMG, BVP_t, skin_t, ECG_t, temp_t, resp_t, SEMG_t)
        rate.sleep()

    rospy.spin() 

if __name__ == '__main__':
    plotter()
