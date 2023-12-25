#! /usr/bin/python3

import rospy
from std_msgs.msg import Float64MultiArray, Int16
import pygame
import tkinter as tk 
from PIL import Image
from PIL import ImageTk  

state = 3
state_prev = 3
curr_state_msg = 'Idle Mode'
one_click = ''
two_click = 'Cursor Control'
three_click = 'Robot Control'
hold_down = ''
instr = 'Robot Off'
audio_path = '/home/akhil/hat_ws/src/hat_pkg/scripts/Audio/idle.wav'


def state_subscriber_callback(d):
    global state_prev, state, curr_state_msg, one_click, two_click, three_click, hold_down, instr, audio_path
    state_prev = state
    state = d.data
    #print(state)
    #print(state_prev)
    pygame.mixer.init()
    if state == 0:
        curr_state_msg = 'Drive Mode'
        one_click = 'Arm Mode'
        two_click = 'Cursor Control'
        three_click = 'Idle Mode'
        hold_down = ''
        instr = 'Robot On'
        audio_path = '/home/akhil/hat_ws/src/hat_pkg/scripts/Audio/drive.wav'
    elif state == 1: 
        curr_state_msg = 'Arm Mode'
        one_click = 'Wrist Mode'
        two_click = 'Cursor Control'
        three_click = 'Idle Mode'
        hold_down = ''
        instr = 'Robot On'
        audio_path = '/home/akhil/hat_ws/src/hat_pkg/scripts/Audio/arm.wav'
    elif state == 2:
        curr_state_msg = 'Wrist Mode'
        one_click = 'Drive Mode'
        two_click = 'Cursor Control'
        three_click = 'Idle Mode'
        hold_down = ''
        instr = 'Robot On'
        audio_path = '/home/akhil/hat_ws/src/hat_pkg/scripts/Audio/wrist.wav'
    elif state == 3: 
        curr_state_msg = 'Idle Mode'
        one_click = ''
        two_click = 'Cursor Control'
        three_click = 'Robot Control'
        hold_down = ''
        instr = 'Robot Off'
        audio_path = '/home/akhil/hat_ws/src/hat_pkg/scripts/Audio/idle.wav'
    elif state == 5:
        curr_state_msg = 'Cursor Control Mode'
        one_click = ''
        two_click = 'Robot Control'
        three_click = 'Idle Mode'
        hold_down = ''
        instr = 'Computer On'
        audio_path = '/home/akhil/hat_ws/src/hat_pkg/scripts/Audio/cursor.wav'
    elif state == 6:
        curr_state_msg = 'Calibration for Robot Control'
        one_click = 'Calibrate HAT'
        two_click = 'Cursor Control'
        three_click = 'Idle Mode'
        hold_down = ''
        instr = 'Please look straight ahead'
        audio_path = '/home/akhil/hat_ws/src/hat_pkg/scripts/Audio/robot_calibration.wav'
    elif state == 7:
        curr_state_msg = 'Calibration for Cursor Control'
        one_click = 'Calibrate HAT'
        two_click = 'Robot Control'
        three_click = 'Idle Mode'
        hold_down = ''
        instr = 'Please look directly at the center of the screen'
        audio_path = '/home/akhil/hat_ws/src/hat_pkg/scripts/Audio/cursor_calibration.wav'

    pygame.mixer.music.load(audio_path)
    pygame.mixer.music.play()


def audio_node(): 
    rospy.init_node("audio_node")
    rospy.Subscriber("state", Int16, state_subscriber_callback)

    
    pygame.mixer.init()
    pygame.mixer.music.load(audio_path)
    pygame.mixer.music.play()
    


if __name__ == '__main__':
    audio_node()
    rospy.spin()

