# Independence in the Home: A Wearable Interface for a Person with Quadriplegia to Teleoperate a Mobile Manipulator
Code for paper: [Independence in the Home: A Wearable Interface for a Person with Quadriplegia to Teleoperate a Mobile Manipulator](https://arxiv.org/abs/2312.15071)\
Authors: Akhil Padmanabha, Janavi Gupta, Chen Chen, Jehan Yang, Vy Nguyen, Doug Weber, Carmel Majidi, Zackory Erickson

## Pre-requisities
### Hardware Architecture:
![System Architecture](https://github.com/RCHI-Lab/HAT2/assets/79553848/7b2935c2-09e5-4778-8ff1-6348239e2229)

- HAT: Follow the bill of materials and circuit diagram on the [website](https://sites.google.com/view/hat2-teleop/home/build-instructions-and-code). Contact [akhilpad@andrew.cmu.edu](mailto:akhilpad@andrew.cmu.edu) for support with build instructions.
- [Stretch RE2 robot](https://docs.hello-robot.com/0.2/.) 
- [Clicker](https://www.amazon.com/V7-Standard-Optical-Notebooks-M30P10-7N/dp/B002FYL7PG?th=1): Any mouse with a piece of tape at the bottom to disallow it from being able to move the cursor (such that only the buttons work) can be used. 
- A laptop
- [Anydesk](https://anydesk.com/en): Or any similar software to gain access to the robot

## Software Installation
### On Laptop
### On Robot
### On Arduino Nano

## Conduct Experiment
![System Overview](https://github.com/RCHI-Lab/HAT2/assets/79553848/7744c84c-7158-4908-a336-cee5a52e92c3)
### Setting threshold values
```bash
  cd hat_ws
  source devel/setup.bash
  roslaunch hat_pkg threshold.launch
```
Click on your clicker once to calibrate the HAT initially. Then, rotate your head to the maximum position that you can along the roll, pitch and yaw axis. Finally, click once more after rotating.
### Launching the system
#### On the laptop
```bash
  cd hat_ws
  source devel/setup.bash
  roslaunch hat_pkg laptop.launch
```
Look at the terminal output initially to ensure that the HAT is calibrated. If it doesn't calibrate, try setting it down on a table or calibrating it like a phone compass by making a figure 8. 
#### On the robot
```bash
cd hat_ws
source devel/setup.bash
```
- With rosbag (check below for more details on rosbag)
  ```bash
    roslaunch hat_pkg robot_data.launch task_name:= <insert task name here>
   ```
- Without rosbag:
  ```bash
    roslaunch hat_pkg robot.launch
  ```
- With driver assistance:
  ```bash
    roslaunch da_core driver_assistance.launch interface:=hat
  ```
### Mode switching
![Mode Switching](https://github.com/RCHI-Lab/HAT2/assets/79553848/9c77cdcc-a507-4d20-aec9-0b9c3b057b4e)

When the script initially launches, the system is in idle mode. Different sequences of clicks using the clicker takes you to different modes. Below is a summary of how mode clicking works:

- Idle mode -> Robot Control (drive mode): 1 click
- Drive mode -> Arm mode -> Wrist mode -> Drive mode: 1 click
- Turning on and off Driver Assistance: 3 clicks
- Idle mode -> Cursor Control ->: 1 click
- Any mode -> Idle mode: Hold down
### Stopping the robot
- To stop the robot from moving temporarily, move your head to the calibrated position.
- To stop HAT from controlling the robot, hold down on the clicker until the system goes into Idle mode.
- If the researcher wants to stop the experiment and stop the robot, kill the terminal on the robot by clicking ```bash Ctrl+C``` or press the E-Stop (white button) on the robot to fully stop it.
  
![EStop button: the white button shown](https://user-images.githubusercontent.com/66550924/194368128-14fd9672-23ec-4a38-b5bf-83271cb101be.png)
### Replaying recorded data
Open three different terminals on the laptop:
- On Terminal 1
  ```bash
    roscore
  ```
- On Terminal 2
  ```bash
    rostopic echo /<topic_name>
  ```
  The following are the topics that are recorded:
  - /rpy_filtered
  - /state
  - /mouse
  - /velocities
  - /da
- On Terminal 3
  ```bash
    cd Desktop/data
    rosbag play <name_of_bag_file>
  ```
