#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32

def sensor():

    rospy.init_node('sensor_node', anonymous=False)
    pub = rospy.Publisher('sensor/value', Int32, queue_size=10)
    rate = rospy.Rate(2) # Hz

    value = Int32()
    value.data = 0

    while not rospy.is_shutdown():

        rospy.loginfo(f"Valor: {value.data}")
        pub.publish(value)
        value.data += 1

        rate.sleep()

if __name__ == "__main__":
    try:
        sensor()
    except rospy.ROSInterruptException:
        pass