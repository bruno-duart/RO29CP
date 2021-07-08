#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Float64, Bool

MAX_VEL_LEFT  = 70 #RPM
MAX_VEL_RIGHT = 70 #RPM

class Motor:
    def __init__(self):
        rospy.init_node("motor_control_node", anonymous=False)
        rospy.Subscriber("control_msg",String, self.cmd_update)
        self.pub_left = rospy.Publisher("motor/velocity/left", Float64,
                queue_size=10)
        self.pub_right = rospy.Publisher("motor/velocity/right", Float64,
                queue_size=10)
        
        self.command = String()
        self.v_motor_right = Float64()
        self.v_motor_left  = Float64()
        self.rate = rospy.Rate(1) #1 Hz

    def cmd_update(self, msg):
        self.command.data = msg.data

    def start(self):
        
        while not rospy.is_shutdown():

            if self.command.data == "forward":
                # avancar
                self.v_motor_right.data  = MAX_VEL_RIGHT
                self.pub_right.publish(self.v_motor_right)
                self.v_motor_left.data  = MAX_VEL_LEFT
                self.pub_left.publish(self.v_motor_left)
            else:
                # desliga os motores
                self.v_motor_right.data  = 0.0
                self.pub_right.publish(self.v_motor_right)
                self.v_motor_left.data  = 0.0
                self.pub_left.publish(self.v_motor_left)

            # para o programa por 1s
            self.rate.sleep()


if __name__ == "__main__":
    try:
        motor = Motor()
        motor.start()
    except rospy.ROSInterruptException:
        pass