#! /usr/bin/python3

from binascii import crc32
import rospy
from std_msgs.msg import Float64MultiArray, Int16
import numpy as np
import pickle
import time 




#choose between roll (0), pitch (1), and yaw (2) for control. remember to change in cursor_control.py also 
cc1 = 1 #stands for control choice
cc2 = 2 #this is the one to change between 0 for roll and 2 for yaw

#control mode (0 is for normal and 1 is for Je-Han's method)
control_mode = 0
last_command = 4
curr_command = 4

use_custom_thresholds = False
if use_custom_thresholds == True:
    filepath = '/home/akhil/hat_ws/src/hat_pkg/scripts/thresholds.pickle'
    with open(filepath, 'rb') as f:
        max_thresholds = pickle.load(f)
        threshold_start_cc1 = 15
        threshold_end_cc1 = max_thresholds[cc1]
        threshold_start_cc2 = 15
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
    threshold_start_cc1 = 15
    threshold_end_cc1 = 40
    threshold_start_cc2 = 15
    threshold_end_cc2 = 40

    cc1_threshold = 20 
    cc2_threshold = 20 


angles = [-420, -420, -420]

base_min_translation_speed = 0 #m/s
base_max_translation_speed = 0.4 #m/s
base_min_rotation_speed = 0 #rad/s
base_max_rotation_speed = 0.4 #rad/s
arm_min_lift_speed = 0 #m/s
arm_max_lift_speed = 0.3 #m/s
arm_min_extend_speed = 0 #m/s
arm_max_extend_speed = 0.13 #m/s
wrist_max_speed = 0.3 #rad/s
wrist_min_speed = 0
gripper_max_speed = 2.0 #rad/s
gripper_min_speed = 0

#assume two seconds till max velocity 
num_secs_to_max = 2
control_rate = 20
base_translation_change = base_max_translation_speed/(control_rate*num_secs_to_max)
base_rotation_change = base_max_rotation_speed/(control_rate*num_secs_to_max)
arm_lift_change = arm_max_lift_speed/(control_rate*num_secs_to_max)
arm_extend_change = arm_max_extend_speed/(control_rate*num_secs_to_max)
wrist_change = wrist_max_speed/(control_rate*num_secs_to_max)
gripper_change = gripper_max_speed/(control_rate*num_secs_to_max)


def interpolation (x, x1, x2, y1, y2):
# x: angle
# x1: threshold
# x2: threshold + range of the angle
# y1: minimum speed
# y2: maximum speed
    y = y1 + ((x - x1)*(y2 - y1)/(x2-x1))
    if y > y2: 
        y = y2
    elif y1 > y:
        y = y1
    return y

def interpolation2(x, x1, x2, y1, y2):
    y = y1 + (angle_difference(x, x1)*(y2-y1))/angle_difference(x2, x1)
    if y > y2: 
        y = y2
    elif y1 > y:
        y = y1
    return y

def interpolation3(x, L1, L2, H1, H2, y1, y2, zero_position):
    #print("Curr Angle: ", x, "Thresholds: ", [H1, L1, zero_position, L2, H2])
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


def hat_subscriber_callback(d):
    global angles
    angles = [d.data[2], d.data[3], d.data[4]]
    #print(angles)

state = 3
def state_subscriber_callback(d):
    global state
    state = d.data


def angle_difference(angle2, angle1 = 0):
    return np.mod(angle2-angle1+180, 360) - 180

def get_thresholds(zero_position, threshold_start, threshold_end):
    L1 = angle_difference(zero_position + threshold_start)
    L2 = angle_difference(zero_position - threshold_start)
    H1 = angle_difference(zero_position + threshold_end )
    H2 = angle_difference(zero_position - threshold_end)
    return L1, L2, H1, H2

'''
zero = -140
t_start = 10
t_end = 50
L1, L2, H1, H2 = get_thresholds(zero, t_start, t_end)
angle = -90
print(interpolation3(angle, L1, L2, H1, H2, 0, 10, zero))


zero = 0
t_start = 10
t_end = 50
L1, L2, H1, H2 = get_thresholds(zero, t_start, t_end)
angle = -50
print(interpolation3(angle, L1, L2, H1, H2, 0, 10, zero))
'''


        



def robot_control_node():
    global state
    rospy.init_node("robot_control")
    rospy.Subscriber("rpy_filtered", Float64MultiArray, hat_subscriber_callback)
    velocity_pub = rospy.Publisher("velocities", Float64MultiArray, queue_size=10)
    rospy.Subscriber("state", Int16, state_subscriber_callback)


    rate = rospy.Rate(control_rate)

    zero_positions = [-420, -420, -420]
    
    #ordering of velocities
    #0 forward/backward (drive), 1 rotation (drive)
    #2 up/down (arm), 3 extend/retract (arm)
    #4 yaw (wrist), 5 open/close (gripper)
    velocities = [0,0,0,0,0,0] 

    curr_command = -420
    last_command = -420
    vel_counter = 10

    while not rospy.is_shutdown():
    
        if state == 6:
            curr_command = -420
            last_command = -420
            zero_positions = angles
            print("Zeroing HAT", angles, "...Waiting for Click")
            #for control mode 0 
           
            cc1_threshold_L1, cc1_threshold_L2, cc1_threshold_H1, cc1_threshold_H2 = get_thresholds(zero_positions[cc1],threshold_start_cc1, threshold_end_cc1)
            cc2_threshold_L1, cc2_threshold_L2, cc2_threshold_H1, cc2_threshold_H2 = get_thresholds(zero_positions[cc2],threshold_start_cc2, threshold_end_cc2)

            #for control mode 1 
            cc1_threshold_L, cc1_threshold_H, _, _ =  get_thresholds(zero_positions[cc1], cc1_threshold, 0)
            cc2_threshold_L, cc2_threshold_H, _, _ =  get_thresholds(zero_positions[cc2], cc2_threshold, 0)





        else:
            if control_mode == 0:
                #note that the default is that all motors are not moving. If the system enters state 0, 1, or 2 only then will there be robot motion
                base_rotational_velocity = 0
                base_translation_velocity = 0
                arm_lift_velocity = 0
                arm_extend_velocity = 0
                wrist_velocity = 0
                gripper_velocity = 0

                # state 0: base movement
                # state 1: arm movement
                # state 2: wrist/gripper
                deltas = np.mod(np.array(zero_positions) - np.array(angles) + 180, 360) - 180
                if state == 0: #base movement
                    if abs(deltas[cc1]) > abs(deltas[cc2]): 
                        base_translation_velocity = -1*interpolation3(angles[cc1], cc1_threshold_L1, cc1_threshold_L2, cc1_threshold_H1, cc1_threshold_H2, base_min_translation_speed, base_max_translation_speed, zero_positions[cc1])               
                    else:
                        base_rotational_velocity = interpolation3(angles[cc2], cc2_threshold_L1, cc2_threshold_L2, cc2_threshold_H1, cc2_threshold_H2, base_min_rotation_speed, base_max_rotation_speed, zero_positions[cc2])

                elif state == 1: #arm movement 
                    if abs(deltas[cc1]) > abs(deltas[cc2]): 
                        arm_lift_velocity = interpolation3(angles[cc1], cc1_threshold_L1, cc1_threshold_L2, cc1_threshold_H1, cc1_threshold_H2, arm_min_lift_speed, arm_max_lift_speed, zero_positions[cc1])               
                    else:
                        arm_extend_velocity = -1*interpolation3(angles[cc2], cc2_threshold_L1, cc2_threshold_L2, cc2_threshold_H1, cc2_threshold_H2, arm_min_extend_speed, arm_max_extend_speed, zero_positions[cc2])

                elif state == 2: #wrist and gripper movement
                    if abs(deltas[cc1]) > abs(deltas[cc2]): 
                        gripper_velocity = interpolation3(angles[cc1], cc1_threshold_L1, cc1_threshold_L2, cc1_threshold_H1, cc1_threshold_H2, gripper_min_speed, gripper_max_speed, zero_positions[cc1])               
                    else:
                        wrist_velocity = interpolation3(angles[cc2], cc2_threshold_L1, cc2_threshold_L2, cc2_threshold_H1, cc2_threshold_H2,wrist_min_speed, wrist_max_speed, zero_positions[cc2])


            elif control_mode == 1:
                last_command = curr_command
                if state == 0 or state == 1 or state == 2:
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
                else:
                    curr_command = 4 

                if curr_command != last_command or curr_command == 4:
                    base_translation_velocity = 0
                    base_rotational_velocity = 0
                    arm_lift_velocity = 0
                    arm_extend_velocity = 0 
                    wrist_velocity = 0
                    gripper_velocity = 0
                #drive mode 
                elif curr_command == 0 and curr_command == last_command and state == 0:
                    base_translation_velocity += base_translation_change 
                    base_translation_velocity = min(base_translation_velocity, base_max_translation_speed)
                elif curr_command == 1 and curr_command == last_command and state == 0:
                    base_translation_velocity -= base_translation_change
                    base_translation_velocity = max(base_translation_velocity, -base_max_translation_speed)
                elif curr_command == 3 and curr_command == last_command and state == 0:
                    base_rotational_velocity += base_rotation_change
                    base_rotational_velocity = min(base_rotational_velocity, base_max_rotation_speed)
                elif curr_command == 2 and curr_command == last_command and state == 0:
                    base_rotational_velocity -= base_rotation_change
                    base_rotational_velocity = max(base_rotational_velocity, -base_max_rotation_speed)
                #arm mode
                elif curr_command == 1 and curr_command == last_command and state == 1:
                    arm_lift_velocity += arm_lift_change 
                    arm_lift_velocity = min(arm_lift_velocity, arm_max_lift_speed)
                elif curr_command == 0 and curr_command == last_command and state == 1:
                    arm_lift_velocity -= arm_lift_change
                    arm_lift_velocity = max(arm_lift_velocity, -arm_max_lift_speed)
                elif curr_command == 2 and curr_command == last_command and state == 1:
                    arm_extend_velocity += arm_extend_change
                    arm_extend_velocity = min(arm_extend_velocity, arm_max_extend_speed)
                elif curr_command == 3 and curr_command == last_command and state == 1:
                    arm_extend_velocity -= arm_extend_change
                    arm_extend_velocity = max(arm_extend_velocity, -arm_max_extend_speed)
                #wrist mode
                elif curr_command == 0 and curr_command == last_command and state == 2:
                    gripper_velocity += gripper_change
                    gripper_velocity = min(gripper_velocity, gripper_max_speed)
                elif curr_command == 1 and curr_command == last_command and state == 2:
                    gripper_velocity -= gripper_change
                    gripper_velocity = max(gripper_velocity, -gripper_max_speed)
                elif curr_command == 2 and curr_command == last_command and state == 2:
                    wrist_velocity += wrist_change
                    wrist_velocity = min(wrist_velocity, wrist_max_speed)
                elif curr_command == 3 and curr_command == last_command and state == 2:
                    wrist_velocity -= wrist_change
                    wrist_velocity = max(wrist_velocity, -wrist_max_speed)

            velocities = [base_translation_velocity,base_rotational_velocity,arm_lift_velocity,arm_extend_velocity,wrist_velocity,gripper_velocity] 
            if vel_counter == 10:
                print("velocities", velocities)
                vel_counter = 0
            else:
                vel_counter += 1
                

        d = Float64MultiArray()
        d.data = velocities
        velocity_pub.publish(d)     

        rate.sleep()

if __name__ == '__main__':
    robot_control_node()
    rospy.spin()
