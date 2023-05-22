import rospy
from car2x_unit.msg import PerceivedObjectContainer, PerceivedObject

def send_message():
    # 创建一个消息对象
    msg = PerceivedObjectContainer()
    msg.numberOfPerceivedObjects = 3
    obj1 = PerceivedObject()
    msg.perceivedObjects.append(obj1)
    obj2 = PerceivedObject()
    msg.perceivedObjects.append(obj2)
    pub.publish(msg)
    
if __name__ == '__main__':
    rospy.init_node('message_sender')
    pub = rospy.Publisher('collective_perception', PerceivedObjectContainer, queue_size=10)
    # pub = rospy.Publisher('collective_perception', PerceivedObjectContainer, queue_size=10)
    rate = rospy.Rate(2)  # 设置发布频率为每秒2次（500ms）

    while not rospy.is_shutdown():
        send_message()
        rate.sleep()