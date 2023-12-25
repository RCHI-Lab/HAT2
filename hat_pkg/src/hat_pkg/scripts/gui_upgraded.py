#! /usr/bin/python3

import rospy
from std_msgs.msg import Float64MultiArray, Int16
import tkinter as tk 
from PIL import Image
from PIL import ImageTk  
import PIL
import cv2
import numpy as np 
import glob 
import time
from sensor_msgs.msg import CompressedImage 
from time import perf_counter

def list_ports():
    """
    Test the ports and returns a tuple with the available ports and the ones that are working.
    """
    non_working_ports = []
    dev_port = 0
    working_ports = []
    available_ports = []
    while len(non_working_ports) < 10: # if there are more than 5 non working ports stop the testing. 
        camera = cv2.VideoCapture(dev_port)
        if not camera.isOpened():
            non_working_ports.append(dev_port)
            print("Port %s is not working." %dev_port)
        else:
            is_reading, img = camera.read()
            w = camera.get(3)
            h = camera.get(4)
            if is_reading:
                print("Port %s is working and reads images (%s x %s)" %(dev_port,h,w))
                working_ports.append(dev_port)
            else:
                print("Port %s for camera ( %s x %s) is present but does not reads." %(dev_port,h,w))
                available_ports.append(dev_port)
        dev_port +=1
    return available_ports,working_ports,non_working_ports

available_ports, working_ports, non_working_ports = list_ports()
print("Working Ports:", working_ports)
if len(working_ports) > 2:
    working_ports = working_ports[-2:]
    print(working_ports)

state = 3
state_prev = 3
curr_state_msg = 'Idle Mode'
one_click = ''
two_click = 'Cursor Control'
three_click = 'Robot Control'
hold_down = ''
image_path_start = '/home/hat/hat_ws/src/hat_pkg/scripts/Images/'
image_path = image_path_start + 'Idle.jpg'
instr = 'Robot Off'
color = 'red'

'''

for camera_path in glob.glob("/dev/video?"):
    c = cv2.VideoCapture(camera_path)
    if c.isOpened() == True:
        print(camera_path)
        result, frame = c.read()
        print(np.shape(frame))
        c.release()
'''


def state_subscriber_callback(d):
    global state_prev, state, curr_state_msg, one_click, two_click, three_click, hold_down, image_path, instr, audio_path, color
    state_prev = state
    state = d.data

    if state == 0:
        curr_state_msg = 'Drive Mode'
        one_click = 'Arm Mode'
        two_click = ' '
        three_click = 'Idle Mode'
        hold_down = 'Drivers Assistance'
        image_path = image_path_start + 'Stretch_Base.jpg'
        instr = 'Robot On'
        color = 'green'
    elif state == 1: 
        curr_state_msg = 'Arm Mode'
        one_click = 'Wrist Mode'
        two_click = ' '
        three_click = 'Idle Mode'
        hold_down = 'Drivers Assistance'
        image_path = image_path_start + 'Stretch_Arm.jpg'
        instr = 'Robot On'
        color = 'green'
    elif state == 2:
        curr_state_msg = 'Wrist Mode'
        one_click = 'Drive Mode'
        two_click = ' '
        three_click = 'Idle Mode'
        hold_down = 'Drivers Assistance'
        image_path = image_path_start + 'Stretch_Gripper.png'
        instr = 'Robot On'
        color = 'green'
    elif state == 3: 
        curr_state_msg = 'Idle Mode'
        one_click = ''
        two_click = 'Cursor Control'
        three_click = 'Robot Control'
        hold_down = ''
        image_path = image_path_start + 'Idle.jpg'
        instr = 'Robot Off'
        color = 'red'
    elif state == 5:
        curr_state_msg = 'Cursor Control Mode'
        one_click = ''
        two_click = ' '
        three_click = 'Idle Mode'
        hold_down = ''
        image_path = image_path_start + 'Computer.jpg'
        instr = 'Computer On'
        color = 'green'
    elif state == 6:
        curr_state_msg = 'Zeroing HAT for Robot Control'
        one_click = 'Calibrate HAT'
        two_click = ' '
        three_click = 'Idle Mode'
        hold_down = ''
        image_path = image_path_start + 'Robot_Zero.jpg'
        instr = 'Please look straight ahead'
        color = 'red'
    elif state == 7:
        curr_state_msg = 'Zeroing HAT for Cursor Control'
        one_click = 'Calibrate HAT'
        two_click = ' '
        three_click = 'Idle Mode'
        hold_down = ''
        image_path = image_path_start + 'Cursor_Zero.png'
        instr = 'Please look directly at the center of the screen'
        color = 'red'




def gui_node(): 
    global state
    rospy.init_node("gui_node")
    mouseclickpub = rospy.Publisher("mouse_clicks", Int16, queue_size = 1)
    nav_camera_pub = rospy.Publisher("nav_cam", CompressedImage, queue_size=5)
    grip_camera_pub = rospy.Publisher("grip_cam", CompressedImage, queue_size=5)
    rospy.Subscriber("state", Int16, state_subscriber_callback)


    window = tk.Tk()
    window.geometry("1920x1200+0+0")
    window.title("HAT GUI")



    def button_release(event):
        m = Int16()
        m.data = 1
        mouseclickpub.publish(m)
    
    def button_click(event):
        m = Int16()
        m.data = 0
        mouseclickpub.publish(m)



    window.bind('<ButtonRelease-1>', button_release)
    window.bind('<Button-1>', button_click)


    font_size_small = 17

    
    #creates a frame for the image and text combination
    frame_img = tk.Frame(window, width = 1500, height = 200)
    frame_img.pack(pady = 50)

    curr_state_label = tk.Label(frame_img, text = " " + curr_state_msg, font = ("Arial", 110), fg = "red", wraplength = 450)
    curr_state_label.grid(row = 0, column = 1, padx = 45, pady = 10)

    image = Image.open(image_path)
    resized_image = image.resize((350, 350))
    photo = ImageTk.PhotoImage(resized_image)
    image_label = tk.Label(frame_img, image = photo, border = 5, borderwidth = 2, relief = 'solid')
    image_label.grid(row = 0, column = 0, padx = 45, pady = 0)

    #creates a frame for the set of instructions embedded in the previous frame
    frame_instr = tk.Frame(frame_img, width = 100, height = 200)
    frame_instr.grid(row = 0, column = 3, padx = 20, pady = 0)

    instr_label = tk.Label(frame_instr, text = " " + instr, font = ("Arial", 30), fg = color, wraplength = 300)
    instr_label.grid(row = 0, column = 0, padx = 10, pady = 30)

    one_click_label_static = tk.Label(frame_instr, text = "One Click ", font = ("Arial", font_size_small, "bold"), justify='left')
    one_click_label_static.grid(row = 1, column = 0, padx = 1, pady = 1, sticky='w')
    one_click_label_dynamic = tk.Label(frame_instr, text = one_click, font = ("Arial", font_size_small), justify='left')
    one_click_label_dynamic.grid(row = 1, column = 1, padx = 1, pady = 1, sticky='w')

    two_click_label_static = tk.Label(frame_instr, text = "Two Clicks: ", font = ("Arial", font_size_small, "bold"), justify='left')
    two_click_label_static.grid(row = 2, column = 0, padx = 1, pady = 1, sticky='w')
    two_click_label_dynamic = tk.Label(frame_instr, text = two_click, font = ("Arial", font_size_small), justify='left')
    two_click_label_dynamic.grid(row = 2, column = 1, padx = 1, pady = 1, sticky='w')

    three_click_label_static = tk.Label(frame_instr, text = "Three Clicks: ", font = ("Arial", font_size_small, "bold"), justify='left')
    three_click_label_static.grid(row = 3, column = 0, padx = 1, pady = 1, sticky='w')
    three_click_label_dynamic = tk.Label(frame_instr, text = three_click, font = ("Arial", font_size_small), justify='left')
    three_click_label_dynamic.grid(row = 3, column = 1, padx = 1, pady = 1, sticky='w')

    hold_down_label_static = tk.Label(frame_instr, text = "Hold Down: ", font = ("Arial", font_size_small, "bold"), justify='left')
    hold_down_label_static.grid(row = 4, column = 0, padx = 1, pady = 1, sticky='w')
    hold_down_label_dynamic = tk.Label(frame_instr, text = hold_down, font = ("Arial", font_size_small), justify='left')
    hold_down_label_dynamic.grid(row = 4, column = 1, padx = 1, pady = 1, sticky='w')


    empty_image = np.zeros([480, 640, 3], dtype = np.uint8).fill(0)
    #empty_image = ImageTk.PhotoImage(image = PIL.Image.fromarray(empty_image))

    camera_frame = tk.Frame(window, width = 1500, height = 200)
    camera_frame.pack(pady = 50)
    
    nav_cam_label = tk.Label(camera_frame, image = empty_image)
    nav_cam_label.config(image = empty_image)
    nav_cam_label.image = empty_image
    nav_cam_label.grid(row = 0, column = 0, columnspan = 3)

    nav_cam = cv2.VideoCapture('/dev/video' + str(working_ports[0])) ###CHANGE THIS 6 to 8 or 8 to 6
    '''
    nav_cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    nav_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    nav_cam.set(cv2.CAP_PROP_FPS, 10)
    nav_cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    '''
    nav_cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)


    result, image = nav_cam.read()
    if result == True:
        #image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        img = ImageTk.PhotoImage(image = PIL.Image.fromarray(image))
        print('Navigation Camera Worked Once')
        #nav_cam_image = canvas.create_image(x1,y1, image = img, anchor = 'nw')
    

    
    gripper_cam_label = tk.Label(camera_frame, image = empty_image)
    gripper_cam_label.config(image = empty_image)
    gripper_cam_label.image = empty_image
    gripper_cam_label.grid(row = 0, column = 3, columnspan = 3)
    
    gripper_cam = cv2.VideoCapture('/dev/video' + str(working_ports[1])) ###CHANGE THIS 6 to 8 or 8 to 6
    '''
    gripper_cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    gripper_cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    gripper_cam.set(cv2.CAP_PROP_FPS, 10)
    gripper_cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    '''
    gripper_cam.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    result, image = gripper_cam.read()
    if result == True:
        img = ImageTk.PhotoImage(image = PIL.Image.fromarray(image))
        print('Gripper Camera Worked Once')

    def update():              
        t1_start = perf_counter()

        curr_state_label.config(text = " " + curr_state_msg, font = ("Arial", 50), fg = "red", wraplength = 450)

        instr_label.config(text = " " + instr, font = ("Arial", 30), fg = color, wraplength = 300)

        one_click_label_static.config(text = "One Click: ", font = ("Arial", font_size_small, "bold"), justify='left')
        one_click_label_dynamic.config(text = one_click, font = ("Arial", font_size_small), justify='left')

        two_click_label_static.config(text = "Two Clicks: ", font = ("Arial", font_size_small, "bold"), justify='left')
        two_click_label_dynamic.config(text = two_click, font = ("Arial", font_size_small), justify='left')

        three_click_label_static.config(text = "Three Clicks: ", font = ("Arial", font_size_small, "bold"), justify='left')
        three_click_label_dynamic.config(text = three_click, font = ("Arial", font_size_small), justify='left')

        hold_down_label_static.config(text = "Hold Down: ", font = ("Arial", font_size_small, "bold"), justify='left')
        hold_down_label_dynamic.config(text = hold_down, font = ("Arial", font_size_small), justify='left')

        image_config = Image.open(image_path)
        resized_image_config = image_config.resize((350, 350))
        photo_config = ImageTk.PhotoImage(resized_image_config)
        image_label.config(image= photo_config, border = 5, borderwidth = 2, relief = 'solid')
        image_label.image = photo_config

        if state == 0 or state == 1 or state == 2: 
            result, image = nav_cam.read()
            if result == True:
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                img = ImageTk.PhotoImage(image = PIL.Image.fromarray(image))
                nav_cam_label.config(image = img)
                nav_cam_label.image = img

            
                msg = CompressedImage()
                msg.header.stamp = rospy.Time.now()
                msg.format='jpeg'
                msg.data=np.array(cv2.imencode('.jpg', image)[1]).tobytes()
                nav_camera_pub.publish(msg)
        else:
            nav_cam_label.config(image = empty_image)
            nav_cam_label.image = empty_image


        if state == 0 or state == 1 or state == 2: 
            result, image = gripper_cam.read()
            if result == True:
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                img = ImageTk.PhotoImage(image = PIL.Image.fromarray(image))
                gripper_cam_label.config(image = img)
                gripper_cam_label.image = img

                msg = CompressedImage()
                msg.header.stamp = rospy.Time.now()
                msg.format='jpeg'
                msg.data=np.array(cv2.imencode('.jpg', image)[1]).tobytes()
                grip_camera_pub.publish(msg)
            else: 
                print('Failed')
            
        else:
            gripper_cam_label.config(image = empty_image)
            gripper_cam_label.image = empty_image
    
        t1_stop = perf_counter()
        #print(f"Elasped time during compressing: {t1_stop - t1_start}")

        window.after(1,update)

    update()
    window.mainloop()


if __name__ == '__main__':
    gui_node()

