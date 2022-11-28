#!/usr/bin/env python3

import sys
import rospy
from std_msgs.msg import Empty

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard.')

def listener(topic_name):

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber(topic_name, Empty, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    topic_name = sys.argv[1]
    listener(topic_name)
