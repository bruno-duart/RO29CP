#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

class Control_two_tb3: # declaração da classe de controle
    def __init__(self):
        rospy.init_node("control_two_tb3_node", anonymous=False) 
        rospy.Subscriber("/cmd_vel", Twist, self.update_vel) # tópico de escuta

        # Tópicos de publicação        
        self.pub_tb3_1 = rospy.Publisher("tb3_0/cmd_vel", Twist, queue_size=10)
        self.pub_tb3_2 = rospy.Publisher("tb3_1/cmd_vel", Twist, queue_size=10)

        self.sleep_rate = rospy.Rate(1) #Hz
        self.twist = Twist()

    def update_vel(self, msg): # atualização de velocidades
        self.twist = msg

    def start(self):
        while not rospy.is_shutdown():
            self.pub_tb3_1.publish(self.twist)
            self.pub_tb3_2.publish(self.twist)
            self.sleep_rate.sleep()

if __name__=="__main__":
    try:
        control_two = Control_two_tb3()
        control_two.start()
    except rospy.ROSInterruptException:
        pass
