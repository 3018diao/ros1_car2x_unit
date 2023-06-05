#!/usr/bin/env python

import rospy
from pdk_ros_msg.msg import pdk_RadarObjectList

def main():
    rospy.init_node('pdk_tracking')
    pub = rospy.Publisher('/pdk/tracking', pdk_RadarObjectList, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz publishing rate


    while not rospy.is_shutdown():
        radar_object_list = pdk_RadarObjectList()
        # Populate the radar_object_list message with data
        # ...

        pub.publish(radar_object_list)
        rate.sleep()

if __name__ == '__main__':
    main()

