#!/usr/bin/env python

import rospy
import serial 
import utm
from driver.msg import gps_msg


def gps_parse():
    gps_pub = rospy.Publisher('gps', gps_msg, queue_size= 10)
    rospy.init_node('gps_parse')
    rate = rospy.Rate(10)
    serial_port = rospy.get_param('port1', '/dev/ttyACM0')
    serial_baud = rospy.get_param('baudrate', '4800')
    port = serial.Serial(serial_port, serial_baud, timeout=3.)

    rospy.loginfo("I am working")

    while not rospy.is_shutdown():
        gps = gps_msg()
        line = port.readline().decode("utf-8")
        if "$GPGGA" in line:
        #if line.startswith("$GPGGA"):
            rospy.loginfo("I am working")
            data = line.split(',')
            gps.header.frame_id= 'GPS1_Frame'
            time = data[1]
            seconds = int(time[0:2])*3600 + int(time[2:4])*60 + int(time[4:6])
            nanoseconds = float(time[7:9])*(10**-9)
            gps.header.stamp.secs = seconds
            gps.header.stamp.nsecs = nanoseconds
            latitude = float(data[2])
            longitude = float(data[4])
            gps.Altitude = float(data[9])

            #conversion to decimal degerees
            LatDeg = int(latitude/100)
            LatSec = latitude - LatDeg * 100
            LongDeg = int(longitude/100)
            LongSec = longitude - LongDeg *100
            
            gps.Latitude = LatDeg + LatSec/60
            gps.Longitude = LongDeg + LongSec/60
            
            if data[5] == "W":
                gps.Longitude *= -1
            if data[3] == "S":
                gps.Longitude *= -1

            utm_measure = utm.from_latlon(gps.Latitude,gps.Longitude)

            gps.UTM_easting = float(utm_measure[0])
            gps.UTM_northing = float(utm_measure[1])
            gps.Zone = int(utm_measure[2])
            gps.Letter = utm_measure[3]
            rospy.loginfo(gps)
            gps_pub.publish(gps)
            rate.sleep()
        
if __name__ == '__main__':
    try:
        gps_parse()
    except rospy.ROSInterruptException:
        pass
