#!/usr/bin/env python
#===============================================================================
#
#===============================================================================

import rospy
import serial
from drive_ros_msgs.msg import mav_RAW_DATA

#===============================================================================
#
#===============================================================================
ser = serial.Serial('/dev/senseboard', 115200, timeout=1)

#===============================================================================
#
#===============================================================================
def to_mav_raw_callback(message):
    rospy.loginfo("Write to mav "+ str(len(message.data)) + " bytes")
    ser.write(message.data)

#===============================================================================
#
#===============================================================================
def node():
    rospy.init_node('mavlink_device_node', anonymous=True)

    pub = rospy.Publisher('/from_mav/mav_raw_data', mav_RAW_DATA, queue_size=10)
    rospy.Subscriber("/to_mav/mav_raw_data", mav_RAW_DATA, to_mav_raw_callback,queue_size=10)

    r = rospy.Rate(1000)

    m = mav_RAW_DATA()
    m.channel=0

    while not rospy.is_shutdown():
      m.data = ser.read(255)
      pub.publish(m)
      r.sleep()

if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException: pass
