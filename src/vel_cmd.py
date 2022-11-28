#!/usr/bin/env python3
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import sys
import rospy
import time
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist

def talker(topic_name):
    rospy.init_node('talker', anonymous=True)
    pub = rospy.Publisher(topic_name, Twist, queue_size=1)

    starttime = time.time()

    while True:
        currenttime = time.time()
        
        if currenttime - starttime > period:
            break

        msg = Twist()
        msg.linear.x = 0
        msg.linear.y = 0
        msg.linear.z = 0.8
        msg.angular.x = 0
        msg.angular.y = 0
        msg.angular.z = 0

        pub.publish(msg)

        time.sleep(0.1 - ((currenttime - starttime) % 0.1))

    print("=========out of loop=================")
    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.linear.z = 0
    msg.angular.x = 0
    msg.angular.y = 0
    msg.angular.z = 0

    pub.publish(msg)
    pub.publish(msg)
    pub.publish(msg)
    pub.publish(msg)

if __name__ == '__main__':
    try:
        topic_name = sys.argv[1]
        period = sys.argv[2]
        period = int(period)
        talker(topic_name)
    except rospy.ROSInterruptException:
        pass
