#!/usr/bin/env python3

import rospy
from std_msgs.msg import String, Float64, Bool

MAX_VEL_LEFT  = 70 #RPM
MAX_VEL_RIGHT = 70 #RPM

class Motor:
    def __init__(self):
        rospy.init_node("motor_control_node", anonymous=False)
        rospy.Subscriber("control_msg", String, self.cmd_update)
        rospy.Subscriber("collision/status", Bool, self.status_update)

        self.pub_left = rospy.Publisher("motor/velocidade/esquerda", Float64,
                queue_size=10)
        self.pub_right = rospy.Publisher("motor/velocidade/direita", Float64,
                queue_size=10)
        
        self.command = String()
        self.status = Bool()
        self.v_motor_right = Float64()
        self.v_motor_left  = Float64()
        self.rate = rospy.Rate(1) #1 Hz

    def cmd_update(self, msg): # atualiza o comando enviado ao motor
        self.command.data = msg.data
        rospy.loginfo(f"command: {self.command.data}")
    
    def status_update(self, msg): # atualiza a leitura do sensor
        self.status.data = msg.data
        rospy.loginfo(f"Colisão detectada? {self.status.data}")

    def start(self):
        
        while not rospy.is_shutdown():
            
            if not self.status.data: # Se não houver resposta positiva sobre colisão, o motor funciona
                if self.command.data == "forward":
                    # avançar
                    self.v_motor_right.data  = MAX_VEL_RIGHT
                    self.pub_right.publish(self.v_motor_right)
                    self.v_motor_left.data  = MAX_VEL_LEFT
                    self.pub_left.publish(self.v_motor_left)
                elif self.command.data == "right":
                    # virar à direita
                    self.v_motor_right.data  = MAX_VEL_RIGHT/4
                    self.pub_right.publish(self.v_motor_right)
                    self.v_motor_left.data  = MAX_VEL_LEFT
                    self.pub_left.publish(self.v_motor_left)
                elif self.command.data == "left":
                    # virar à esquerda
                    self.v_motor_right.data  = MAX_VEL_RIGHT
                    self.pub_right.publish(self.v_motor_right)
                    self.v_motor_left.data  = MAX_VEL_LEFT/4
                    self.pub_left.publish(self.v_motor_left)
                elif self.command.data == "backward":
                    # marcha ré
                    self.v_motor_right.data  = -MAX_VEL_RIGHT
                    self.pub_right.publish(self.v_motor_right)
                    self.v_motor_left.data  = -MAX_VEL_LEFT
                    self.pub_left.publish(self.v_motor_left)
                else:
                    # desliga os motores
                    self.v_motor_right.data  = 0.0
                    self.pub_right.publish(self.v_motor_right)
                    self.v_motor_left.data  = 0.0
                    self.pub_left.publish(self.v_motor_left)
                rospy.loginfo(f"ML: {self.v_motor_left} -- MR: {self.v_motor_right}")
            else:
                # desliga os motores por colisão
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