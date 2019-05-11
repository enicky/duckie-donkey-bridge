#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy

def handle_msg(msg):
    rospy.loginfo('handling message')
    print('handling message : ', msg)


rospy.init_node('test_joy_subsciber')

subscriber_topic_name = "/duckie/joy"

rospy.loginfo("subscribing to %s" % subscriber_topic_name)

rospy.Subscriber(subscriber_topic_name, Joy, handle_msg)
print("starting spin")
rospy.spin()
