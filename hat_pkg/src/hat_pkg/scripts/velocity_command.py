#! /usr/bin/python3

import rospy
from std_msgs.msg import Float64MultiArray
import numpy as np
import stretch_body.robot as rb
import stretch_body.robot
import stretch_body.stepper
import stretch_body.end_of_arm
import time

r =stretch_body.robot.Robot()
r.startup()
assert(r.is_calibrated()) # the robot must be homed

#r.end_of_arm.get_joint('wrist_yaw').set_velocity(0.1)
#time.sleep(5)
#r.end_of_arm.get_joint('wrist_yaw').set_velocity(0)

#r.end_of_arm.get_joint('wrist_yaw').set_velocity(0.1)
#r.end_of_arm.move_by('wrist_yaw', 0.2)

'''
def enable_velmode_on_dxl(motor):
    motor.disable_torque()
    motor.enable_vel()
    motor.enable_torque()
enable_velmode_on_dxl(r.end_of_arm.get_joint('wrist_yaw').motor)
enable_velmode_on_dxl(r.end_of_arm.get_joint('stretch_gripper').motor)

r.end_of_arm.get_joint('wrist_yaw').motor.set_vel(50) #ticks/s
'''

#ordering of velocities
#0 forward/backward (drive), 1 rotation (drive)
#2 up/down (arm), 3 extend/retract (arm)
#4 yaw (wrist), 5 open/close (gripper)
velocities = [0,0,0,0,0,0] 

def velocity_subscriber_callback(d):
    global velocities 
    velocities = d.data
    

def velocity_command_node():
    global velocities
    rospy.init_node("velocity_command")
    rospy.Subscriber("velocities", Float64MultiArray, velocity_subscriber_callback)

    control_rate = 50
    rate = rospy.Rate(control_rate)

    while not rospy.is_shutdown():
        r.base.set_velocity(velocities[0], velocities[1])
        r.lift.set_velocity(velocities[2])
        r.arm.set_velocity(velocities[3])
        r.end_of_arm.get_joint('wrist_yaw').set_velocity(velocities[4])
        r.end_of_arm.get_joint('stretch_gripper').set_velocity(velocities[5])
        #r.end_of_arm.get_joint('wrist_yaw').motor.set_vel(velocities[4]) #ticks/s
        #r.end_of_arm.get_joint('stretch_gripper').motor.set_vel(velocities[5]) #ticks/s
        r.push_command()
        rate.sleep()
        
    



if __name__ == '__main__':
    velocity_command_node()
    rospy.spin()
