#!/usr/bin/env python3

# modified from https://github.com/ros-teleop/teleop_twist_keyboard/blob/master/teleop_twist_keyboard.py

from __future__ import print_function

import threading

import roslib

roslib.load_manifest("teleop_twist_keyboard")

import sys
from select import select

import rospy
from std_msgs.msg import Float64MultiArray
from velocity_commander import VelocityCommander

if sys.platform == "win32":
    import msvcrt
else:
    import termios
    import tty


msg = """
---------------------------
Moving around:
       w
    a  s  d

Moving arm:
       i
    j  k  l

Turning gripper:
    u     o

anything else : stop

z/x : increase/decrease only linear speed by 10%
c/v : increase/decrease only angular speed by 10%

CTRL-C to quit
"""

moveBindings = {
    "w": (1, 0, 0, 0, 0),
    "s": (-1, 0, 0, 0, 0),
    "a": (0, 1, 0, 0, 0),
    "d": (0, -1, 0, 0, 0),
    "i": (0, 0, 1, 0, 0),
    "k": (0, 0, -1, 0, 0),
    "j": (0, 0, 0, -1, 0),
    "l": (0, 0, 0, 1, 0),
    "u": (0, 0, 0, 0, 1),
    "o": (0, 0, 0, 0, -1),
}

speedBindings = {
    "z": (1.1, 1),
    "x": (0.9, 1),
    "c": (1, 1.1),
    "v": (1, 0.9),
}


class PublishThread(threading.Thread):
    def __init__(self, rate):
        super(PublishThread, self).__init__()
        self.publisher = rospy.Publisher(
            "/teleop_velocity_command", Float64MultiArray, queue_size=1
        )
        self.x = 0.0
        self.th = 0.0
        self.lift = 0.0
        self.arm_ext = 0.0
        self.wrist = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.condition = threading.Condition()
        self.done = False

        # Set timeout to None if rate is 0 (causes new_message to wait forever
        # for new data to publish)
        if rate != 0.0:
            self.timeout = 1.0 / rate
        else:
            self.timeout = None

        self.start()

    def wait_for_subscribers(self):
        i = 0
        while not rospy.is_shutdown() and self.publisher.get_num_connections() == 0:
            if i == 4:
                print("Waiting for subscriber to connect to topic")
            rospy.sleep(0.5)
            i += 1
            i = i % 5
        if rospy.is_shutdown():
            raise Exception("Got shutdown request before subscribers connected")

    def update(self, x, th, lift, arm_ext, wrist, speed, turn):
        self.condition.acquire()
        self.x = x
        self.th = th
        self.lift = lift
        self.arm_ext = arm_ext
        self.wrist = wrist
        self.speed = speed
        self.turn = turn
        # Notify publish thread that we have a new message.
        self.condition.notify()
        self.condition.release()

    def stop(self):
        self.done = True
        self.update(0, 0, 0, 0, 0, 0, 0)
        self.join()

    def run(self):
        while not self.done:
            self.condition.acquire()
            # Wait for a new message or timeout.
            self.condition.wait(self.timeout)

            # Copy state into twist message.
            x = self.x * self.speed
            th = self.th * self.turn
            lift = self.lift * self.speed
            single_arm_ext = self.arm_ext * self.speed / 4
            wrist = self.wrist * self.turn

            self.condition.release()

            # Publish.
            self.publisher.publish(
                Float64MultiArray(data=[x, th, lift, *[single_arm_ext] * 4, wrist])
            )

        # Publish stop message when thread exits.
        self.publisher.publish(Float64MultiArray(data=[0] * 8))


def getKey(settings, timeout):
    if sys.platform == "win32":
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        rlist, _, _ = select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def saveTerminalSettings():
    if sys.platform == "win32":
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == "win32":
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)


if __name__ == "__main__":
    settings = saveTerminalSettings()

    rospy.init_node("teleop_twist_keyboard")

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    speed_limit = rospy.get_param("~speed_limit", 1000)
    turn_limit = rospy.get_param("~turn_limit", 1000)
    repeat = rospy.get_param("~repeat_rate", 0.0)
    key_timeout = rospy.get_param("~key_timeout", 0.1)
    twist_frame = rospy.get_param("~frame_id", "")

    pub_thread = PublishThread(repeat)

    x = 0
    lift = 0
    arm_ext = 0
    wrist = 0
    th = 0.0
    status = 0

    try:
        pub_thread.wait_for_subscribers()
        pub_thread.update(x, th, lift, arm_ext, wrist, speed, turn)

        print(msg)
        print(vels(speed, turn))
        while 1:
            key = getKey(settings, key_timeout)
            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                lift = moveBindings[key][2]
                arm_ext = moveBindings[key][3]
                wrist = moveBindings[key][4]
            elif key in speedBindings.keys():
                speed = min(speed_limit, speed * speedBindings[key][0])
                turn = min(turn_limit, turn * speedBindings[key][1])
                if speed == speed_limit:
                    print("Linear speed limit reached!")
                if turn == turn_limit:
                    print("Angular speed limit reached!")
                print(vels(speed, turn))
                if status == 14:
                    print(msg)
                status = (status + 1) % 15
            else:
                # Skip updating cmd_vel if key timeout and robot already
                # stopped.
                if key == "" and x == 0 and th == 0 and lift == 0 and arm_ext == 0 and wrist == 0:
                    continue
                x, th, lift, arm_ext, wrist = [0] * 5
                if key == "\x03":
                    break

            pub_thread.update(x, th, lift, arm_ext, wrist, speed, turn)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restoreTerminalSettings(settings)
