import rospy
from cpm_interfaces.msg import PerceivedObjectContainer, PerceivedObject, CartesianPosition3dWithConfidence, CartesianCoordinateWithConfidence

def send_message():

    # msg = PerceivedObjectContainer()
    # msg.numberOfPerceivedObjects = 3
    # obj1 = PerceivedObject()
    # msg.perceivedObjects.append(obj1)
    # obj2 = PerceivedObject()
    # msg.perceivedObjects.append(obj2)

    perceivedObjectContainer = PerceivedObjectContainer()
    perceivedObjectContainer.numberOfPerceivedObjects = 1
    perceivedObject = PerceivedObject()
    perceivedObject.objectID = 1
    perceivedObject.a = 0

    cartesianPosition3dWithConfidence = CartesianPosition3dWithConfidence()

    x_cord_cartesianCoordinateWithConfidence = CartesianCoordinateWithConfidence()
    x_cord_cartesianCoordinateWithConfidence.value = 3

    y_cord_cartesianCoordinateWithConfidence = CartesianCoordinateWithConfidence()
    y_cord_cartesianCoordinateWithConfidence.value = 2

    cartesianPosition3dWithConfidence.has_z_cord = False
    cartesianPosition3dWithConfidence.x_cord = x_cord_cartesianCoordinateWithConfidence
    cartesianPosition3dWithConfidence.y_cord = y_cord_cartesianCoordinateWithConfidence

    perceivedObject.position = cartesianPosition3dWithConfidence
    perceivedObjectContainer.perceivedObjects.append(perceivedObject)

    pub.publish(perceivedObjectContainer)
    
if __name__ == '__main__':
    rospy.init_node('message_sender')
    pub = rospy.Publisher('collective_perception', PerceivedObjectContainer, queue_size=10)
    # pub = rospy.Publisher('collective_perception', PerceivedObjectContainer, queue_size=10)
    rate = rospy.Rate(2)  

    while not rospy.is_shutdown():
        send_message()
        rate.sleep()