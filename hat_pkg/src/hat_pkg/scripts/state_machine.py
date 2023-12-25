#! /usr/bin/python3
import rospy
from std_msgs.msg import Int16
import serial.tools.list_ports as port_list
import serial 
import time
import numpy as np



mouse_event = 0 #1 is single click, 2 is double click, 3 is triple click, 4 is hold down,  
state = 3 
da = 0


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
    global state, mouse_event, da
    rospy.init_node("state_machine")
    state_pub = rospy.Publisher("state", Int16, queue_size=1)
    da_pub = rospy.Publisher("da", Int16, queue_size=1)
    mouse_sub = rospy.Subscriber("mouse", Int16, mouse_sub_callback)

    sampling_rate = 20
    rate = rospy.Rate(sampling_rate)
    state_change = False
    da_change = False
    while not rospy.is_shutdown():
        #mode switching with single clicks
        if state == 0 and mouse_event == 1: 
            state = 1
            print("Switching from Drive Mode to Arm Mode")
            state_change = True
            mouse_event = 0
        elif state == 1 and mouse_event == 1:
            state = 2
            print("Switching from Arm Mode to Wrist Mode")
            state_change = True
            mouse_event = 0
        elif state == 2 and mouse_event == 1:
            state = 0 
            print("Switching from Wrist Mode to Drive Mode")
            state_change = True
            mouse_event = 0
        #switching to and from idle state
        elif (state == 0 or state == 1 or state == 2 or state == 4 or state == 5 or state == 6 or state == 7) and mouse_event == 3:
            state = 3 
            print("Turning off HAT")
            state_change = True
            mouse_event = 0
        #switching to zero mode for robot control 
        elif (state == 3 and mouse_event == 3): # or ((state == 5 or state == 7) and mouse_event == 2):
            state = 6 
            print("Turning on HAT and waiting for calibration click")
            state_change = True
            mouse_event = 0
        #finished zero mode for robot control and switching to drive 
        elif state == 6 and mouse_event == 1: 
            state = 0
            print("Zeroed and in Drive Mode")
            state_change = True
            mouse_event = 0         
        #switching to zero mode for computer control
        elif state == 3 and mouse_event == 2:#(state == 0 or state == 1 or state == 2 or state == 3 or state == 6) and mouse_event == 2:
            state = 7
            print("Switching to cursor control and waiting for calibration click")
            state_change = True
            mouse_event = 0
        #finished zero mode for computer control and switching to computer control 
        elif state == 7 and mouse_event == 1: 
            state = 5
            print("Zeroed and in Cursor Mode")
            state_change = True
            mouse_event = 0 
        #driver assistance 
        elif (state == 0 or state == 1 or state == 2) and da == 0 and mouse_event == 4:
            print("Turning on driver assistance")
            da = 1
            da_change = True
            mouse_event = 0 
        elif (state == 0 or state == 1 or state == 2) and da == 1 and mouse_event == 4:
            print("Turning off driver assistance")
            da = 0
            da_change = True
            mouse_event = 0 


        if state_change == True:
            d = Int16()
            d.data = state
            state_pub.publish(d)
            state_change = False

        if da_change == True:
            d = Int16()
            d.data = da
            da_pub.publish(d)
            da_change = False
        rate.sleep()
    
if __name__ == '__main__':
    try:
        state_machine_node()
    except rospy.ROSInterruptException:
        pass