#! /usr/bin/python3
import rospy
from std_msgs.msg import Int16
import serial.tools.list_ports as port_list
import serial 
import time
import numpy as np



mouse_event = 0 #1 is single click, 2 is double click, 3 is triple click, 4 is hold down,  
state = 9
state_change = True

# state 0: base movement
# state 1: arm movement
# state 2: wrist/gripper
# state 3: idle
# state 4: calibrate 
# state 5: cursor control
# state 6: robot zero mode
# state 7: cursor control zero mode  
# state 8: threshold calibration
# state 9: threshold calibration zero mode 
# state 10: save file for threshold calibration 


def mouse_sub_callback(d):
    global mouse_event 
    mouse_event = d.data
    

def state_machine_node():
    global state, mouse_event
    rospy.init_node("state_machine")
    state_pub = rospy.Publisher("state", Int16, queue_size=1)
    mouse_sub = rospy.Subscriber("mouse", Int16, mouse_sub_callback)

    sampling_rate = 20
    rate = rospy.Rate(sampling_rate)
    state_change = False
    while not rospy.is_shutdown():
        #switching from zero mode to threshold collection
        if (state == 9) and (mouse_event == 1):
            state = 8
            print("Collecting max angles")
            state_change = True
            mouse_event = 0
        #finished zero mode for robot control and switching to drive 
        elif state == 8 and mouse_event == 1: 
            state = 10
            print("In saving mode")
            state_change = True
            mouse_event = 0         


        if state_change == True:
            d = Int16()
            d.data = state
            state_pub.publish(d)
            state_change = False
        rate.sleep()
    
if __name__ == '__main__':
    try:
        state_machine_node()
    except rospy.ROSInterruptException:
        pass