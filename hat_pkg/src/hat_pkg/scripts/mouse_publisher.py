#! /usr/bin/python3
import rospy
from std_msgs.msg import Int16
import serial.tools.list_ports as port_list
import serial 
import time
import numpy as np
#from pynput import mouse
import pygame


(width, height) = (400, 250)
screen = pygame.display.set_mode((width, height))
pygame.display.set_caption('Title')
screen.fill((255,255,255))
pygame.display.flip()    
running = True
clock = pygame.time.Clock()

def mouse_publish():
    rospy.init_node("mouse_publisher")
    mouse_pub = rospy.Publisher("mouse", Int16, queue_size=1)

    sampling_rate = 30
    rate = rospy.Rate(sampling_rate)


    mouse_state = 'unpressed'
    mouse_press = []
    mouse_release = []
    reload_time = 0.5
    last_event_time = rospy.get_time()
    ignore_release = False
    message = 0 #1 is single click, 2 is double click, 3 is triple click, 4 is hold down 
    while not rospy.is_shutdown():
        curr_time = rospy.get_time()
        for event in pygame.event.get():
            if event.type == pygame.MOUSEBUTTONDOWN and len(mouse_press) == 0:
                last_event_time = curr_time
                mouse_press.append(curr_time)
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mouse_press.append(curr_time)
            elif event.type == pygame.MOUSEBUTTONUP and ignore_release == False:
                mouse_release.append(curr_time)
            elif event.type == pygame.MOUSEBUTTONUP:
                ignore_release = False

        #print(mouse_press, mouse_release, curr_time - last_event_time)
        if (curr_time - last_event_time > 1.0):
            if len(mouse_press) == 1 and len(mouse_release) == 0:
                #print("Holding down button")
                last_event_time = curr_time
                mouse_press = []
                mouse_release = []
                ignore_release = True
                message = 4
            elif len(mouse_press) == 1 and len(mouse_release) == 1:
                #print("Single Press")
                last_event_time = curr_time
                mouse_press = []
                mouse_release = []
                message = 1
            elif len(mouse_press) == 2 and len(mouse_release) == 2:
                #print("Double Press")
                last_event_time = curr_time
                mouse_press = []
                mouse_release = []
                message = 2
            elif len(mouse_press) >= 3 and len(mouse_release) >= 3:
                #print("Triple Press")
                last_event_time = curr_time
                mouse_press = []
                mouse_release = []
                message = 3

        if message != 0:
            #publish 
            m = Int16()
            m.data = message
            mouse_pub.publish(m)
            message = 0

        clock.tick(30) # capped at 30 fps
        rate.sleep()

if __name__ == '__main__':
    try:
        mouse_publish()
    except rospy.ROSInterruptException:
        pass

        