#!/usr/bin/env python3

import sys
import rospy
from std_msgs.msg import Empty

def talker(topic_name):
    pub = rospy.Publisher(topic_name, Empty, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    pub.publish()

    # rate = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():
    #     hello_str = "hello world %s" % rospy.get_time()
    #     rospy.loginfo(hello_str)
    #     pub.publish(hello_str)
    #     rate.sleep()

if __name__ == '__main__':
    try:
        topic_name = sys.argv[1]
        talker(topic_name)
    except rospy.ROSInterruptException:
        pass
