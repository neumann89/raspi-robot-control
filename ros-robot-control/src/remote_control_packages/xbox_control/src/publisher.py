#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import xbox
import time
from motor_control_msg.msg import Motor


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + ": Motor callback, command=%i, vel_left=%f,vel_right=%f", 
                  data.command, data.velocity_left, data.velocity_right)

    if data.command is 0:
        amspi.stop_dc_motors([amspi.DC_Motor_1, amspi.DC_Motor_2])
    else:
        speed_left = None if abs(data.velocity_left) >= 100 else abs(data.velocity_left)
        amspi.run_dc_motors([amspi.DC_Motor_1], clockwise=(data.velocity_left >= 0), speed=speed_left)

        speed_right = None if abs(data.velocity_right) >= 100 else abs(data.velocity_right)
        amspi.run_dc_motors([amspi.DC_Motor_2], clockwise=(data.velocity_right >= 0), speed=speed_right)

def init():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('xbox_control_publisher', anonymous=True)

    # Run publisher
    pub = rospy.Publisher("motor", Motor, queue_size=1)
    motor_enable = 0
    msg = Motor()
    msg.command = 0
    msg.velocity_left = 0
    msg.velocity_right = 0

    rospy.loginfo(rospy.get_caller_id() + ": Publisher started.")

    joy = xbox.Joystick()
    rospy.loginfo(rospy.get_caller_id() + ": Xbox controller started.")

    while not joy.Back():
        if joy.leftY():
            msg.command = 1
            msg.velocity_left = 100.0 * joy.leftY()
            pub.publish(msg)
        if joy.rightY():
            msg.command = 1
            msg.velocity_right = 100.0 * joy.rightY()
            pub.publish(msg)
        if joy.A():
            msg.command = 0
            msg.velocity_left = 0
            msg.velocity_right = 0
            pub.publish(msg)
        time.sleep(0.3)

        

    # spin() simply keeps python from exiting until this node is stopped
    #rospy.spin()

if __name__ == '__main__':
    init()
