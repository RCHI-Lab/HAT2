#! /usr/bin/python3

import rospy
from std_msgs.msg import Float64MultiArray, Int16
import numpy as np
import pickle



angles = [-420, -420, -420]
all_deltas = []


def interpolation (x, x1, x2, y1, y2):
# x: angle
# x1: threshold
# x2: threshold + range of the angle
# y1: minimum speed
# y2: maximum speed
    y = y1 + ((x - x1)*(y2 - y1)/(x2-x1));
    if y > y2: 
        y = y2
    elif y1 > y:
        y = y1
    return y


def hat_subscriber_callback(d):
    global angles
    angles = [d.data[2], d.data[3], d.data[4]]
    #print(angles)

state = 9
def state_subscriber_callback(d):
    global state
    state = d.data
    
threshold_saved = False

def threshold_node():
    global state, threshold_saved
    rospy.init_node("threshold_node")
    rospy.Subscriber("rpy_filtered", Float64MultiArray, hat_subscriber_callback)
    rospy.Subscriber("state", Int16, state_subscriber_callback)

    sampling_rate = 20 
    rate = rospy.Rate(sampling_rate)

    zero_positions = [-420, -420, -420]
    

    while (not threshold_saved) and (not rospy.is_shutdown()):
    
        if state == 9:
            zero_positions = angles
            print("Zeroing HAT", angles, "...Waiting for Click")
        elif state == 8:
            #add values to an array and take the max / min from the zero positions
            deltas = np.abs(np.array(zero_positions) - np.array(angles))
            all_deltas.append(deltas)
            max_deltas = np.max(np.array(all_deltas), axis = 0)
            print("Max Deltas:", "  Roll: ",  np.round(max_deltas[0]), "  Pitch: ",  np.round(max_deltas[1]), "  Yaw: ",  np.round(max_deltas[2]), "...click to continue")
        elif state == 10: 
            if np.max(max_deltas) > 100:
                print()
                print()
                print("Not Valid Try Threshold Calibration Again")
                threshold_saved = True
            else:
                print()
                print()
                print("Saved Max Delta", max_deltas)            
                #save file locally 
                threshold_saved = True
                filepath = '/home/akhil/hat_ws/src/hat_pkg/scripts/thresholds.pickle'
                with open(filepath, 'wb') as handle:
                    pickle.dump(max_deltas, handle)
             

        rate.sleep()

if __name__ == '__main__':
    threshold_node()
