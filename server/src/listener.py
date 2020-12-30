#!/usr/bin/env python
import rospy
from std_msgs.msg import Int16

value = 0

def callback(x):
    global value
    value = x.data

if __name__ == '__main__':
    rospy.init_node('listener',anonymous =True)

    rospy.Subscriber("chatter", Int16, callback)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        rospy.loginfo(value)
        rate.sleep()