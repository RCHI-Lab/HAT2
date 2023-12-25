#! /usr/bin/python3
import rospy
from std_msgs.msg import Int16
import serial.tools.list_ports as port_list
import serial 
import time
import numpy as np


mouse_click = -1
mouse_press = []
mouse_release = []
ignore_release = False
last_event_time = 0

def mouse_callback(d):
    global mouse_click, mouse_press, mouse_release, ignore_release, last_event_time
    mouse_click = d.data
    print(mouse_click)
    curr_time = rospy.get_time()
    if mouse_click == 0 and len(mouse_press) == 0:
        last_event_time = curr_time
        mouse_press.append(curr_time)
        mouse_click = -1
    elif mouse_click == 0:
        mouse_press.append(curr_time)
        mouse_click = -1
    elif mouse_click == 1 and ignore_release == False:
        mouse_release.append(curr_time)
        mouse_click = -1
    elif mouse_click == 1:
        ignore_release = False
        mouse_click = -1

def mouse_publish():
    global mouse_click, mouse_press, mouse_release, ignore_release, last_event_time
    rospy.init_node("mouse_publisher")
    mouse_pub = rospy.Publisher("mouse", Int16, queue_size=1)
    mouse_click_sub = rospy.Subscriber("mouse_clicks", Int16, mouse_callback, queue_size=2)

    sampling_rate = 30
    rate = rospy.Rate(sampling_rate)

    mouse_click_time = 1.5 ##CHANGE THIS AS NEEDED
    last_event_time = rospy.get_time()
    message = 0 #1 is single click, 2 is double click, 3 is triple click, 4 is hold down 
    while not rospy.is_shutdown():
        curr_time = rospy.get_time()
    

        #print(len(mouse_press), len(mouse_release), curr_time - last_event_time, ignore_release)
        if (curr_time - last_event_time > mouse_click_time):
            if len(mouse_press) == 1 and len(mouse_release) == 0:
                print("Holding down button")
                last_event_time = curr_time
                mouse_press = []
                mouse_release = []
                ignore_release = True
                message = 4
            elif len(mouse_press) == 1 and len(mouse_release) == 1:
                print("Single Press")
                last_event_time = curr_time
                mouse_press = []
                mouse_release = []
                message = 1
            elif len(mouse_press) == 2 and len(mouse_release) == 2:
                print("Double Press")
                last_event_time = curr_time
                mouse_press = []
                mouse_release = []
                message = 2
            elif len(mouse_press) >= 3 and len(mouse_release) >= 3:
                print("Triple Press")
                last_event_time = curr_time
                mouse_press = []
                mouse_release = []
                message = 3
            elif len(mouse_press) > 1 or len(mouse_release) > 1:
                print('Clearing')
                mouse_press = []
                mouse_release = []

        if message != 0:
            #publish 
            m = Int16()
            m.data = message
            mouse_pub.publish(m)
            message = 0

        rate.sleep()

if __name__ == '__main__':
    try:
        mouse_publish()
    except rospy.ROSInterruptException:
        pass

        