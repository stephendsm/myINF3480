#!/usr/bin/env python

import math
import rospy
from in3140_msgs.msg import RobotPos


def talker():
    pub = rospy.Publisher('/coords', RobotPos, queue_size=10)
    rospy.init_node('coords_publisher', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    position = RobotPos()
    position.name = "Our Awsome Robot"
    position.pos.z = 1

    ctr = 0
    while not rospy.is_shutdown():
        position.pos.x = math.cos(ctr)
        position.pos.y = math.sin(ctr)
        ctr += 0.2
        rospy.loginfo(position)  # This is only for debugging
        pub.publish(position)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
