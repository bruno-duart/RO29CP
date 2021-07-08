#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def callback(msg):

    rospy.loginfo(f"Ouvi: {msg.data}")

def app():

    rospy.init_node('app_node', anonymous=True)

    rospy.Subscriber('sensor/value', Int32, callback)

    rospy.spin()



if __name__ == "__main__":
    try:
        app()
    except rospy.ROSInterruptException:
        pass