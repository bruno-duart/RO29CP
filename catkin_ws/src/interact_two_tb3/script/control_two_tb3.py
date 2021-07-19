#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class Control_two_tb3:
    def __init__(self):
        rospy.init_node("control_two_tb3_node", anonymous=False)
        rospy.Subscriber("/cmd_vel", Twist, self.update_vel)
        
        self.pub_tb3_1 = rospy.Publisher("tb3_0/cmd_vel", Twist, queue_size=10)
        self.pub_tb3_2 = rospy.Publisher("tb3_1/cmd_vel", Twist, queue_size=10)

        self.twist = Twist()

    def update_vel(self, msg):
        self.twist = msg
        #rospy.loginfo(self.twist)

    def start(self):
        while not rospy.is_shutdown():
            self.pub_tb3_1.publish(self.twist)
            self.pub_tb3_2.publish(self.twist)

if __name__=="__main__":
    try:
        control_two = Control_two_tb3()
        control_two.start()
    except rospy.ROSInterruptException:
        pass
