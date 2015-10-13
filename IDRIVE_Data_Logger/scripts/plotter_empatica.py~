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

bvp = np.zeros(1)
acc_x = np.zeros(1)
acc_y = np.zeros(1)
acc_z = np.zeros(1)
gsr = np.zeros(1)
ibi = np.zeros(1)
hr = np.zeros(1)
tmp = np.zeros(1)

bvp_t = np.zeros(1)
acc_t = np.zeros(1)
gsr_t = np.zeros(1)
ibi_t = np.zeros(1)
hr_t = np.zeros(1)
tmp_t = np.zeros(1)

#this function plots the datas
def plot_simulation(bvp_temp, acc_x_temp, acc_y_temp, acc_z_temp, gsr_temp, ibi_temp, hr_temp, tmp_temp, bvp_t, acc_t, gsr_t, ibi_t, hr_t, tmp_t):
    hfmt = dates.DateFormatter("%H:%M:%S.%f")
    fig1 = plt.figure(1,figsize=(16,10))
    ax1 = plt.subplot(321)
    ax1.clear()
    ax1.plot_date(bvp_t, bvp_temp, 'r')
    ax1.set_ylabel('Blood volume pulse')
    ax1.xaxis.set_major_formatter(hfmt)
    for tick in ax1.xaxis.get_major_ticks():
        tick.label.set_fontsize(8)
    ax1.grid('on')
    ax2 = plt.subplot(323)
    ax2.clear()
    ax2.plot_date(gsr_t, gsr_temp, 'r')
    ax2.set_ylabel('Galvanic skin response')
    ax2.xaxis.set_major_formatter(hfmt)
    for tick in ax2.xaxis.get_major_ticks():
        tick.label.set_fontsize(8)
    ax2.grid('on')
    ax3 = plt.subplot(325)
    ax3.clear()
    ax3.plot_date(tmp_t, tmp_temp, 'r')
    ax3.set_ylabel('Temperature')
    ax3.xaxis.set_major_formatter(hfmt)
    for tick in ax3.xaxis.get_major_ticks():
        tick.label.set_fontsize(8)
    ax3.set_ylim(30,40)
    ax3.grid('on')
    ax4 = plt.subplot(322)
    ax4.clear()
    ax4.plot_date(ibi_t, ibi_temp, 'r')
    ax4.set_ylabel('Interbeat interval')
    ax4.xaxis.set_major_formatter(hfmt)
    for tick in ax4.xaxis.get_major_ticks():
        tick.label.set_fontsize(8)
    ax4.grid('on')
    ax5 = plt.subplot(324)
    ax5.clear()
    ax5.plot_date(hr_t, hr_temp, 'r')
    ax5.set_ylabel('Heart rate')
    ax5.xaxis.set_major_formatter(hfmt)
    for tick in ax5.xaxis.get_major_ticks():
        tick.label.set_fontsize(8)
    ax5.grid('on')
    ax6 = plt.subplot(326)
    ax6.clear()
    ax6.plot_date(acc_t, acc_x_temp, 'r')
    ax6.plot_date(acc_t, acc_y_temp, 'b')
    ax6.plot_date(acc_t, acc_z_temp, 'g')
    ax6.set_ylabel('Accelleration')
    ax6.xaxis.set_major_formatter(hfmt)
    for tick in ax5.xaxis.get_major_ticks():
        tick.label.set_fontsize(8)
    ax6.grid('on')
    plt.draw()
    plt.show(block=False)

#this function read the topic "empatica_sensor"
def real_update(data):
    global bvp, acc_x, acc_y, acc_z, gsr, ibi, hr, tmp, bvp_t, acc_t, gsr_t, ibi_t, hr_t, tmp_t
    
    if (data.sensor_type == 'Galvanic Skin Response'):
	if (len (data.data)> 0):
           gsr = np.append(gsr[1:],data.data)
           gsr_t = np.append(gsr_t[1:], dates.date2num(datetime.utcfromtimestamp(data.header.stamp.to_sec())))		
    elif (data.sensor_type == 'Accelleration'):
	if (len (data.data)> 0):
          acc_x = np.append(acc_x[1:],data.data[0])
          acc_y = np.append(acc_y[1:],data.data[1])
          acc_z = np.append(acc_z[1:],data.data[2])
          acc_t = np.append(acc_t[1:], dates.date2num(datetime.utcfromtimestamp(data.header.stamp.to_sec())))
    elif (data.sensor_type == 'Temperature'):
	if (len (data.data)> 0):
           tmp = np.append(tmp[1:],data.data)
	   tmp_t = np.append(tmp_t[1:], dates.date2num(datetime.utcfromtimestamp(data.header.stamp.to_sec())))
    elif (data.sensor_type == 'BVP'):
	if (len (data.data)> 0):
       	   bvp = np.append(bvp[1:],data.data)
    	   bvp_t = np.append(bvp_t[1:], dates.date2num(datetime.utcfromtimestamp(data.header.stamp.to_sec())))
    elif (data.sensor_type == 'Interbeat Interval'):
	if (len (data.data)> 0):
           ibi = np.append(ibi[1:],data.data)
	   ibi_t = np.append(ibi_t[1:], dates.date2num(datetime.utcfromtimestamp(data.header.stamp.to_sec())))
    elif (data.sensor_type == 'Heart Rate'):
	if (len (data.data)> 0):
           hr = np.append(hr[1:],data.data)
	   hr_t = np.append(hr_t[1:], dates.date2num(datetime.utcfromtimestamp(data.header.stamp.to_sec())))

def plotter():
    global bvp, acc_x, acc_y, acc_z, gsr, ibi, hr, tmp, bvp_t, acc_t, gsr_t, ibi_t, hr_t, tmp_t
    
    rospy.init_node('plotter', anonymous=True)
    plotter_seconds = rospy.get_param('~seconds_window')

    bvp = np.zeros(64 * plotter_seconds)
    acc_x = np.zeros(32 * plotter_seconds)
    acc_y = np.zeros(32 * plotter_seconds)
    acc_z = np.zeros(32 * plotter_seconds)
    gsr = np.zeros(4 * plotter_seconds)
    ibi = np.zeros(4 * plotter_seconds)
    hr = np.zeros(4 * plotter_seconds)
    tmp = np.zeros(4 * plotter_seconds)
    dts64 = map(datetime.utcfromtimestamp, np.zeros(64 * plotter_seconds))
    dts32 = map(datetime.utcfromtimestamp, np.zeros(32 * plotter_seconds))
    dts4 = map(datetime.utcfromtimestamp , np.zeros(4 * plotter_seconds))
    bvp_t = dates.date2num(dts64)
    acc_t = dates.date2num(dts32)
    gsr_t = dates.date2num(dts4)
    ibi_t = dates.date2num(dts4)
    hr_t = dates.date2num(dts4)
    tmp_t = dates.date2num(dts4)

    rospy.Subscriber("empatica_sensor", Bio_sensor,real_update)

    delta_t = rospy.get_param('~delta_time')
    rate = rospy.Rate(1/delta_t)
    
    while not rospy.is_shutdown():
        # Plots	
	np.round(bvp,6)
	np.round(tmp,2)
	np.round(gsr,6)
	np.round(ibi,6)
	np.round(acc_x,3)
	np.round(acc_y,3)
	np.round(acc_z,3)
        plot_simulation(bvp, acc_x, acc_y, acc_z, gsr, ibi, hr, tmp, bvp_t, acc_t, gsr_t, ibi_t, hr_t, tmp_t)
        rate.sleep()

    rospy.spin() 

if __name__ == '__main__':
    plotter()
