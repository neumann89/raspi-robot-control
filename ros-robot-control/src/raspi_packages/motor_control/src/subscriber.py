#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from motor_control_msg.msg import Motor
from AMSpi import AMSpi

amspi = None

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

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    # Init Arduino Motor Shield
    global amspi
    amspi = AMSpi.AMSpi()
    amspi.set_74HC595_pins(17, 18, 22)          # Set PINs for controlling shift register (GPIO numbering)
    amspi.set_L293D_pins(None, None, 23, 24)    # Set PINs for controlling the 4 possible motors (GPIO numbering)
    amspi.stop_dc_motors([amspi.DC_Motor_1, amspi.DC_Motor_2])
    rospy.loginfo(rospy.get_caller_id() + ": Motor started.")

    # Run subscriber
    rospy.Subscriber("motor", Motor, callback)
    rospy.loginfo(rospy.get_caller_id() + ": Subscriber started.")

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
