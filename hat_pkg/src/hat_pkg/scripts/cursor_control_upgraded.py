#! /usr/bin/python3

import rospy
from std_msgs.msg import Float64MultiArray, Int16
import numpy as np
from pynput.mouse import Button, Controller
import scipy
import pickle

mouse = Controller()

#CHANGE THIS choose between roll (0), pitch (1), and yaw (2) for control. remember to change in robot_control.py also 
cc1 = 1
cc2 = 2



def angle_difference(angle2, angle1 = 0):
    return np.mod(angle2-angle1+180, 360) - 180

def interpolation3(x, L1, L2, H1, H2, y1, y2, zero_position):
    if angle_difference(zero_position,x) < 0: 
        x1 = L1
        x2 = H1
        sign = -1
    else:
        x1 = L2
        x2 = H2 
        sign = 1
    y = y1 + (angle_difference(x, x1)*(y2-y1))/angle_difference(x2, x1)
    if y > y2: 
        y = y2
    elif y1 > y:
        y = y1
    return sign*y  


def get_thresholds(zero_position, threshold_start, threshold_end):
    L1 = angle_difference(zero_position + threshold_start)
    L2 = angle_difference(zero_position - threshold_start)
    H1 = angle_difference(zero_position + threshold_end )
    H2 = angle_difference(zero_position - threshold_end)
    return L1, L2, H1, H2


use_custom_thresholds = False
if use_custom_thresholds == True:
    filepath = '/home/akhil/hat_ws/src/hat_pkg/scripts/thresholds.pickle'
    with open(filepath, 'rb') as f:
        max_thresholds = pickle.load(f)
        threshold_start_cc1 = 10
        threshold_end_cc1 = max_thresholds[cc1]
        threshold_start_cc2 = 10
        threshold_end_cc2 = max_thresholds[cc2]

        if threshold_end_cc1 > 40: 
            threshold_end_cc1 = 40
            threshold_start_cc1 = 10
            cc1_threshold = 20
        else:
            cc1_threshold = threshold_end_cc1 / 2

        if threshold_end_cc2 > 40:
            threshold_end_cc2 = 40
            threshold_start_cc2 = 10
            cc2_threshold = 20
        else:        
            cc2_threshold = threshold_end_cc2 / 2 

        print("Thresholds:", threshold_start_cc1, threshold_end_cc1, threshold_start_cc2, threshold_end_cc2)



else:

    threshold_start_cc1 = 10
    threshold_end_cc1 = 35

    threshold_start_cc2= 10
    threshold_end_cc2 = 35

    cc1_threshold = 20
    cc2_threshold = 20


#control mode (0 is for normal and 1 is for Je-Han's method)
control_mode = 0 #CHANGE THIS

angles = [-420, -420, -420]

cc2_max_vel = 15
cc2_min_vel = 0

cc1_max_vel = 15
cc1_min_vel = 0




last_command = 4
curr_command = 4

#assume two seconds till max velocity 
num_secs_to_max = 3
control_rate = 20
cc1_change = cc1_max_vel/(control_rate*num_secs_to_max)
cc2_change = cc2_max_vel/(control_rate*num_secs_to_max)




def hat_subscriber_callback(d):
    global angles
    angles = [d.data[2], d.data[3], d.data[4]]
    #print(angles)

state = 3
def state_subscriber_callback(d):
    global state
    state = d.data


def cursor_control_node():
    global angles
    rospy.init_node("cursor_control")
    rospy.Subscriber("rpy_filtered", Float64MultiArray, hat_subscriber_callback)
    rospy.Subscriber("state", Int16, state_subscriber_callback)


    rate = rospy.Rate(control_rate)

    zero_positions = [-420, -420, -420]
    cc2_speed = 0
    cc1_speed = 0 
    curr_command = -420
    last_command = -420

    while not rospy.is_shutdown():
        if state == 7:
            if angles[0] != -420 and angles[1] !=-420 and angles[2] !=-420:
                zero_positions = angles
                print("Zeroing HAT", angles, "...Waiting for Click")

                #for alternate control mode 
                cc1_threshold_L = zero_positions[cc1] - cc1_threshold
                cc1_threshold_H = zero_positions[cc1] + cc1_threshold
                cc2_threshold_L = zero_positions[cc2] - cc2_threshold
                cc2_threshold_H = zero_positions[cc2] + cc2_threshold

                #for original control mode 
                cc1_threshold_L1, cc1_threshold_L2, cc1_threshold_H1, cc1_threshold_H2 = get_thresholds(zero_positions[cc1],threshold_start_cc1, threshold_end_cc1)
                cc2_threshold_L1, cc2_threshold_L2, cc2_threshold_H1, cc2_threshold_H2 = get_thresholds(zero_positions[cc2],threshold_start_cc2, threshold_end_cc2)

        else:
            if control_mode == 0 and state == 5:

                cc1_speed = -1*interpolation3(angles[cc1], cc1_threshold_L1, cc1_threshold_L2, cc1_threshold_H1, cc1_threshold_H2, cc1_min_vel, cc1_max_vel, zero_positions[cc1])               
                cc2_speed = -1*interpolation3(angles[cc2], cc2_threshold_L1, cc2_threshold_L2, cc2_threshold_H1, cc2_threshold_H2, cc2_min_vel, cc2_max_vel, zero_positions[cc2])               

           
            elif control_mode == 1 and state == 5: 
                last_command = curr_command
                
                if abs(zero_positions[cc1]-angles[cc1]) > abs(zero_positions[cc2]-angles[cc2]): #if pitch angle is greater than yaw
                    if angles[cc1] > cc1_threshold_H:
                        curr_command = 0
                    elif angles[cc1] < cc1_threshold_L:
                        curr_command = 1
                    else:
                        curr_command = 4
                elif abs(zero_positions[cc1]-angles[cc1]) < abs(zero_positions[cc2]-angles[cc2]):
                    if angles[cc2] > cc2_threshold_H:
                        curr_command = 2
                    elif angles[cc2] < cc2_threshold_L:
                        curr_command = 3
                    else: 
                        curr_command = 4
                print(last_command, curr_command, cc1_speed, cc2_speed)
                if curr_command != last_command or curr_command == 4:
                    cc2_speed = 0
                    cc1_speed = 0
                elif curr_command == 0 and curr_command == last_command:
                    cc1_speed += cc1_change
                    cc1_speed = min(cc1_speed, cc1_max_vel) 
                elif curr_command == 1 and curr_command == last_command:
                    cc1_speed -= cc1_change
                    cc1_speed = max(cc1_speed, -cc1_max_vel) 
                elif curr_command == 2 and curr_command == last_command:
                    cc2_speed += cc2_change
                    cc2_speed = min(cc2_speed, cc2_max_vel) 
                elif curr_command == 3 and curr_command == last_command:
                    cc2_speed -= cc2_change
                    cc2_speed = max(cc2_speed, -cc2_max_vel) 
                
            else: 
                cc2_speed = 0
                cc1_speed = 0
                
        mouse.move(cc2_speed, cc1_speed)
        rate.sleep()
    
if __name__ == '__main__':
    cursor_control_node()
    rospy.spin()



    