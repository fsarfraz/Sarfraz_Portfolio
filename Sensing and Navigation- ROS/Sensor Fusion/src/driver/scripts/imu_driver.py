#!/usr/bin/env python

import rospy
import serial
import numpy as np 
from driver.msg import imu_msg



def imu_parse():
    imu_pub = rospy.Publisher('imu', imu_msg, queue_size= 10)
    rospy.init_node('imu_parse')
    rate = rospy.Rate(10)
    serial_port = rospy.get_param('port', '/dev/ttyACM0')
    serial_baud = rospy.get_param('baudrate', '115200')
    #serial_port1 = '/dev/pts/2'
    port = serial.Serial(serial_port, serial_baud, timeout=3.)
    imu = imu_msg()
    imu.header.seq = 0

    rospy.loginfo("I am working")

    while not rospy.is_shutdown():
        line = port.readline().decode("utf-8")
        rospy.loginfo(line)
        if "$VNYMR" in line:
            rospy.loginfo("I am working")
            now = rospy.get_rostime()
            imu.header.stamp.secs = now.secs
            imu.header.stamp.nsecs = now.nsecs
            imu.header.frame_id= 'IMU1_Frame'

            data = line.split(',')
    
            yaw = float(data[1])
            pitch = float(data[2])
            roll = float(data[3])
        
            imu.IMU.orientation.x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
            imu.IMU.orientation.y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
            imu.IMU.orientation.z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
            imu.IMU.orientation.w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

            imu.MagField.magnetic_field.x = float(data[4])
            imu.MagField.magnetic_field.y = float(data[5])
            imu.MagField.magnetic_field.z = float(data[6])

            imu.IMU.linear_acceleration.x = float(data[7])
            imu.IMU.linear_acceleration.y = float(data[8])
            imu.IMU.linear_acceleration.z = float(data[9])
            
            string = data[12]
            
            imu.IMU.angular_velocity.x = float(data[10])
            imu.IMU.angular_velocity.y = float(data[11])
            imu.IMU.angular_velocity.z = float(string[0:10])
        
            imu.header.seq += 1
            rospy.loginfo(imu)
            imu_pub.publish(imu)
            rate.sleep()
        
if __name__ == '__main__':
    try:
        imu_parse()
    except rospy.ROSInterruptException:
        pass
