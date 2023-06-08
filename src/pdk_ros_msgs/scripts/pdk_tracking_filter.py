#!/usr/bin/env python
import rospy
from pdk_ros_msgs.msg import pdk_RadarObjectList
from cpm_interfaces.msg import *


pub_filtered = rospy.Publisher('filtered_data_topic', pdk_RadarObjectList, queue_size=10)
def callback(data: pdk_RadarObjectList):
    if data.f_ObjectScore > 0.7 and data.f_DistX < 100 and data.f_DistY < 50:
    # Create a new message for the filtered data
       
       # Publish the filtered data to the new topic
        perceivedObjectContainer = PerceivedObjectContainer()
        
        perceivedObjectContainer.numberOfPerceivedObjects = 1
        perceivedObject = PerceivedObject()
        perceivedObject.objectID = data.u_ObjId

        cartesianPosition3dWithConfidence = CartesianPosition3dWithConfidence()

        x_cord_cartesianCoordinateWithConfidence = CartesianCoordinateWithConfidence()
        x_cord_cartesianCoordinateWithConfidence.value = data.f_DistX

        y_cord_cartesianCoordinateWithConfidence = CartesianCoordinateWithConfidence()
        y_cord_cartesianCoordinateWithConfidence.value = data.f_DistY

        cartesianPosition3dWithConfidence.has_z_cord = False
        cartesianPosition3dWithConfidence.x_cord = x_cord_cartesianCoordinateWithConfidence
        cartesianPosition3dWithConfidence.y_cord = y_cord_cartesianCoordinateWithConfidence

        perceivedObject.position = cartesianPosition3dWithConfidence
        perceivedObjectContainer.perceivedObjects.append(perceivedObject)
        

        pub_filtered.publish(perceivedObjectContainer)
  

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/pdk/tracking', pdk_RadarObjectList, callback)
    rospy.spin()


if __name__ == '__main__':
 
 listener()

