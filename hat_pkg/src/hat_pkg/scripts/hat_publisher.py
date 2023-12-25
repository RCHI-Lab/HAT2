#! /usr/bin/python3
import rospy
from std_msgs.msg import Float64MultiArray
import serial.tools.list_ports as port_list
import serial 
import time
import numpy as np


def hat_publish():
    rospy.init_node("hat_publisher")
    rpy_pub = rospy.Publisher("rpy", Float64MultiArray, queue_size=1)
    rpy_filtered_pub = rospy.Publisher("rpy_filtered", Float64MultiArray, queue_size=1)

    sampling_rate = 20
    rate = rospy.Rate(sampling_rate)

    
    ports = list(port_list.comports())
    arduino_port = 0
    for p in ports:
        #f p.device[0:3] == 'COM': ###UNCOMMENT this LINE and comment the next IF USING WINDOWS###
        if p.device[0:11] == '/dev/ttyUSB':
            arduino_port = p.device 
            print(p.device)
    
    #arduino_port = '/dev/ttyUSB3' #remove this later and add code above to automatically find 
    if arduino_port == 0: 
        print('Arduino not found. Check connection.')
        exit()

    ser = serial.Serial(arduino_port, baudrate = 115200, timeout = 1)
    print("Established serial link with HAT")
    buffer = []
    rolls = []
    pitchs = []
    yaws = []
    times = []
    roll = 0
    pitch = 0
    yaw = 0
    num_values = 5
    sys = 0
    gyro = 0
    accel = 0
    mag = 0
    angle_count = 10
    printed = False
    while not rospy.is_shutdown():
        if ser.in_waiting:
            buffer.append(ser.read())
            if len(buffer) >= 25:
                byte0 = int.from_bytes(buffer[0], byteorder='little')
                byte1 = int.from_bytes(buffer[1],byteorder='little')
                byte2 = int.from_bytes(buffer[2],byteorder='little')
                if byte0 == 255 and byte1 == 255 and byte2 == 255:
                    x = int.from_bytes(buffer[3] + buffer[4],byteorder='little')/100 #yaw
                    y = int.from_bytes(buffer[5] + buffer[6],byteorder='little')/100 - 180 #pitch
                    z = int.from_bytes(buffer[7] + buffer[8],byteorder='little')/100 - 180 #roll
                    sys = int.from_bytes(buffer[9] + buffer[10],byteorder='little')
                    gyro = int.from_bytes(buffer[11] + buffer[12],byteorder='little')
                    accel = int.from_bytes(buffer[13] + buffer[14],byteorder='little')
                    mag = int.from_bytes(buffer[15] + buffer[16],byteorder='little')
                    curr_time = int.from_bytes(buffer[17] + buffer[18] + buffer[19] + buffer[20],byteorder='little')/1e6

                    #print(curr_time)
                    #print(round(x,2),round(y,2),round(z,2), round(micro_times[-1]-micro_times[-2], 5), round(py_times[-1]-py_times[-2],3))
                    
                    #adjust the yaw to -180 to 180 instead of 0 to 360
                    if x > 180:
                        x = x - 360

                    local_time = rospy.get_time()

                    #publish unfiltered data 
                    d = Float64MultiArray()
                    d.data = [local_time, curr_time, z, y, x]
                    rpy_pub.publish(d)

                    #filter data 
                    times.append(local_time)
                    rolls.append(z)
                    pitchs.append(y)
                    yaws.append(x)
                    '''
                    if len(times)>1: 
                        #get last roll, pitch, and yaw 
                        last_roll = rolls[-1]
                        last_pitch = pitchs[-1]
                        last_yaw = yaws[-1]
                    
                        #calculate the difference between the last and current roll, pitch, and yaw
                        roll_diff = z - last_roll
                        pitch_diff = y - last_pitch
                        yaw_diff = x - last_yaw

                        if roll_diff < -180:
                            z += 360
                        if roll_diff > 180:
                            z-=360
                        if pitch_diff < -180:
                            y += 360
                        if pitch_diff > 180:
                            y-=360
                        if yaw_diff < -180:
                            x += 360
                        if yaw_diff > 180:
                            x-=360
                    '''
                    if gyro == 3 and mag == 3 and printed == False: 
                        print("HAT Calibrated!", sys, gyro, accel, mag)
                        printed = True
                    elif printed == False: 
                        print(sys, gyro, accel, mag)
                    if len(times) < num_values and sys != 3 and mag !=3:
                        print("Roll:", z, "  Pitch", y, "  Yaw:", x)
                        print(sys, gyro, accel, mag)
                        roll = -420 #not enough values collected yet
                        pitch = -420 #not enough values collected yet
                        yaw = -420 #not enough values collected yet
                    else:
                        #wait for the first "num_values" values to arrive and then perform outlier rejection using a median 
                        roll = np.median(rolls)
                        pitch = np.median(pitchs) 
                        yaw = np.median(yaws)

                        if angle_count == 10:
                            print("hat_angles", roll, pitch, yaw)
                            angle_count = 0
                        else:
                            angle_count += 1

                        #pop until the lists are only "num_values" length
                        while len(times) > num_values: 
                            times.pop(0)
                            rolls.pop(0)
                            pitchs.pop(0)
                            yaws.pop(0)

                    #publish filtered data 
                    d = Float64MultiArray()
                    d.data = [local_time, curr_time, roll, pitch, yaw]
                    rpy_filtered_pub.publish(d)      
                    
                buffer.pop(0)
        else:
            rate.sleep()

if __name__ == '__main__':
    try:
        hat_publish()
    except rospy.ROSInterruptException:
        pass