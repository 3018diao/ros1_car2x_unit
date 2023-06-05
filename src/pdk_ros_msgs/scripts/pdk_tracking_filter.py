#!/usr/bin/env python
import rospy
from pdk_ros_msgs.msg import pdk_RadarObjectList

pub_filtered = rospy.Publisher('filtered_data_topic', pdk_RadarObjectList, queue_size=10)
def callback(data):
    if data.f_ObjectScore > 0.7 and data.f_DistX < 100 and data.f_DistY < 50:
    # Create a new message for the filtered data
        filtered_message = pdk_RadarObjectList ()
        filtered_message.header = data.header
        filtered_message.u_ObjId = data.u_ObjId
        filtered_message.f_DistX = data.f_DistX
        filtered_message.f_DistY = data.f_DistY
        filtered_message.f_VrelX = data.f_VrelX
        filtered_message.f_VrelY = data.f_VrelY
        filtered_message.f_ArelX = data.f_ArelX
        filtered_message.f_ArelY = data.f_ArelY
        filtered_message.f_DistXStd = data.f_DistXStd
        filtered_message.f_DistYStd = data.f_DistYStd
        filtered_message.f_VrelXStd = data.f_VrelXStd
        filtered_message.f_VrelYStd = data.f_VrelYStd
        filtered_message.f_ArelXStd = data.f_ArelXStd
        filtered_message.f_ArelYStd = data.f_ArelYStd
        filtered_message.f_LDeltaX_left = data.f_LDeltaX_left
        filtered_message.f_LDeltaX_mid = data.f_LDeltaX_mid
        filtered_message.f_LDeltaX_right = data.f_LDeltaX_right
        filtered_message.f_LDeltaY_left = data.f_LDeltaY_left
        filtered_message.f_LDeltaY_mid = data.f_LDeltaY_mid
        filtered_message.f_LDeltaY_right = data.f_LDeltaY_right
        filtered_message.f_RCS = data.f_RCS
        filtered_message.f_ObjectScore = data.f_ObjectScore
        filtered_message.u_LifeCycles = data.u_LifeCycles
        filtered_message.f_VabsX = data.f_VabsX
        filtered_message.f_VabsY = data.f_VabsY
        filtered_message.f_AabsX = data.f_AabsX
        filtered_message.f_AabsY = data.f_AabsY
        filtered_message.f_VabsXStd = data.f_VabsXStd
        filtered_message.f_VabsYStd = data.f_VabsYStd
        filtered_message.f_AabsXStd = data.f_AabsXStd
        filtered_message.f_AabsYStd = data.f_AabsYStd
        
        # Print the filtered data attributes
        #print("filtered_message.u_ObjId:", data.u_ObjId)
        #print("filtered_message.f_ObjectScore:", data.f_ObjectScore)
        #print("filtered_message.f_DistX:", data.f_DistX)
        #print("filtered_message.f_DistY:", data.f_DistY)
       
       # Publish the filtered data to the new topic
        pub_filtered.publish(filtered_message)
  

def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber('/pdk/tracking', pdk_RadarObjectList, callback)
    rospy.spin()


if __name__ == '__main__':
 
 listener()

