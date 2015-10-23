#!/usr/bin/env python
# license removed for brevity
import rospy
import socket
import nmea_msgs.msg
from nmea_msgs.msg import Sentence
from sensor_msgs.msg import Imu
import std_msgs.msg

GYRO_SCALE_FACTOR = 0.09766
TEMP_SCALE_FACTOR = 0.1453
TEMP_SUM_FACTOR = 25
ACCEL_SCALE_FACTOR = 0.001221

CONVERT_MSEC2 = 9.81
CONVERT_RADSEC = 0.01745329251994

UDP_IP = "255.255.255.255"
UDP_PORT = 8308

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))


def velodyne_Dat():
    nmea = rospy.Publisher('nmea_sentence', Sentence, queue_size=1)
    imu = rospy.Publisher('imu_data', Imu, queue_size=1)
    rospy.init_node('velodyne_GPS', anonymous=True)
    r = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        data, addr = sock.recvfrom(1206) # buffer size is 1206 bytes
        if (addr[0] == "192.168.1.201"): # Ensure message is from Velodyne HDL32-E
            val = data[206:278].encode('ascii')
	    gyro1_temp =(data[15:16] + data[14:15]).encode("hex")
	    gyro1 = int( gyro1_temp[1:],16)
	    if gyro1 > 2047:
    		gyro1 = (-4096 + gyro1)
	    gyro1 = gyro1 * GYRO_SCALE_FACTOR
	    print "gyro1 ",gyro1
	    temp1_temp =(data[17:18] + data[16:17]).encode("hex")
	    temp1 = int( temp1_temp[1:],16)
	    if temp1 > 2047:
    		temp1 = (-4096 + temp1)
	    temp1 = (temp1 * TEMP_SCALE_FACTOR) + TEMP_SUM_FACTOR
	    print "temp1 ",temp1
	    accel1X_temp =(data[19:20] + data[18:19]).encode("hex")
	    accel1X = int( accel1X_temp[1:],16)
	    if  accel1X > 2047:
    		 accel1X = (-4096 +  accel1X)
  	    accel1X = accel1X * ACCEL_SCALE_FACTOR
	    print "accel1X ",accel1X
	    accel1Y_temp =(data[21:22] + data[20:21]).encode("hex")
	    accel1Y = int( accel1Y_temp[1:],16)
	    if  accel1Y > 2047:
    		 accel1Y = (-4096 +  accel1Y)
	    accel1Y = accel1Y * ACCEL_SCALE_FACTOR
	    print "accel1Y ",accel1Y 
	    gyro2_temp =(data[23:24] + data[22:23]).encode("hex")
	    gyro2 = int( gyro2_temp[1:],16)
	    if gyro2 > 2047:
    		gyro2 = (-4096 + gyro2)
	    gyro2 = gyro2 * GYRO_SCALE_FACTOR
	    print "gyro2 ",gyro2
	    temp2_temp =(data[25:26] + data[24:25]).encode("hex")
	    temp2 = int( temp2_temp[1:],16)
	    if temp2 > 2047:
    		temp2 = (-4096 + temp2)
	    temp2 = (temp2 * TEMP_SCALE_FACTOR) + TEMP_SUM_FACTOR
	    print "temp2 ",temp2
	    accel2X_temp =(data[27:28] + data[26:27]).encode("hex")
	    accel2X = int( accel2X_temp[1:],16)
	    if  accel2X > 2047:
    		 accel2X = (-4096 +  accel2X)
  	    accel2X = accel2X * ACCEL_SCALE_FACTOR
	    print "accel2X ",accel2X
	    accel2Y_temp =(data[29:30] + data[28:29]).encode("hex")
	    accel2Y = int( accel2Y_temp[1:],16)
	    if  accel2Y > 2047:
    		 accel2Y = (-4096 +  accel2Y)
	    accel2Y = accel2Y * ACCEL_SCALE_FACTOR
	    print "accel2Y ",accel2Y 
	    gyro3_temp =(data[31:32] + data[30:31]).encode("hex")
	    gyro3 = int( gyro3_temp[1:],16)
	    if gyro3 > 2047:
    		gyro3 = (-4096 + gyro3)
	    gyro3 = gyro3 * GYRO_SCALE_FACTOR
	    print "gyro3 ",gyro3
	    temp3_temp =(data[33:34] + data[32:33]).encode("hex")
	    temp3 = int( temp3_temp[1:],16)
	    if temp3 > 2047:
    		temp3 = (-4096 + temp3)
	    temp3 = (temp3 * TEMP_SCALE_FACTOR) + TEMP_SUM_FACTOR
	    print "temp3 ",temp3
	    accel3X_temp =(data[35:36] + data[34:35]).encode("hex")
	    accel3X = int( accel3X_temp[1:],16)
	    if  accel3X > 2047:
    		 accel3X = (-4096 +  accel3X)
  	    accel3X = accel3X * ACCEL_SCALE_FACTOR
	    print "accel3X ",accel3X
	    accel3Y_temp =(data[37:38] + data[36:37]).encode("hex")
	    accel3Y = int( accel3Y_temp[1:],16)
	    if  accel3Y > 2047:
    		 accel3Y = (-4096 +  accel3Y)
	    accel3Y = accel3Y * ACCEL_SCALE_FACTOR
	    print "accel3Y ",accel3Y 
	    print '-------------------------------------------------------------------'
            #print val
            #print data[206:278].encode("hex")
            h = std_msgs.msg.Header()
            h.stamp = rospy.Time.now()
            h.frame_id = "velodyne" 
            Sen = nmea_msgs.msg.Sentence()
            Sen.header = h
            Sen.sentence = val # "$GPRMC,192326,A,4324.2427,N,08028.2217,W,000.0,000.0,080914,009.7,W,D*1F"
	    imu_msg = Imu();
	    imu_msg.header = h
	    imu_msg.linear_acceleration.x = ((accel1X + accel2X) / 2) * CONVERT_MSEC2
            imu_msg.linear_acceleration.y = ((accel2Y + accel3Y) / 2)   * CONVERT_MSEC2
	    imu_msg.linear_acceleration.z = ((accel3X + (-accel1Y)) / 2) * CONVERT_MSEC2
	    imu_msg.angular_velocity.x = -gyro3 * CONVERT_RADSEC
	    imu_msg.angular_velocity.y = gyro1 * CONVERT_RADSEC
	    imu_msg.angular_velocity.z = gyro2 * CONVERT_RADSEC
	    imu.publish(imu_msg)
            nmea.publish(Sen)
            r.sleep()

if __name__ == '__main__':
    try:
        velodyne_Dat()
    except rospy.ROSInterruptException: pass
    

