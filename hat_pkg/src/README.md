# HAT Package Installation
* Obtain the IP of the robot and laptop
  ```bash
  ip a
  ```
## Installation on the Robot
* On Stretch RE2, create a new user account `<account_name>` and log into that account. You can use [AnyDesk](https://anydesk.com/en) to obtain remote desktop control of the robot
* Create a folder called `hat_ws` in this new user account
* Create a folder called `src` in the `hat_ws` folder
* Clone the HAT repository to  `hat_ws/src` in your `<account_name>` account on your Stretch RE2
  ```bash
    git clone https://github.com/RCHI-Lab/HAT2.git
    cd HAT2
    git sparse-checkout init
    git sparse-checkout set "hat_pkg/src/"
    ```
  If you use git version older than 2.25.0, please follow [this link](https://stackoverflow.com/questions/600079/how-do-i-clone-a-subdirectory-only-of-a-git-repository) to manually setup the sparse checkout.
* Under the `scripts` folder install the following requirements:
    ```bash
  pip3 install numpy
  pip3 install time
  pip3 install glob
  pip3 install tkinter
  pip3 install opencv-python
  pip3 install pillow
  ```
* Make the workspace
  ```bash
  catkin_make
  ```
* Add the robot and the laptop IP to the hosts file
  ```bash
  sudo nano /etc/hosts
  ```
  * Add the following lines:
  ```bash
  <robot_ip> robot
  <laptop_ip> laptop
  ```
* Update the `bashrc`
  ```bash
  nano ~/.bashrc
  ```
  * Add the following lines to the `bashrc`:
    ```bash
    source /home/<account_name>/catkin_ws/devel/setup.bash
    export ROS_MASTER_URI=http://laptop:11311
    export ROS_HOSTNAME=<robot_ip> 
    ```
* Close the current terminal
* Source the updated `bashrc`
  ```bash
  source devel/setup.bash
  ```
* Ensure that the laptop is on the network
    ```bash
    ping laptop
    ```    
## Installation on the Laptop
* On your laptop, create a new user account `<account_name>` and log into that account.
* Create a folder called `hat_ws` in this new user account
* Create a folder called `src` in the `hat_ws` folder
* Clone the HAT repository to  `hat_ws/src` in your `<account_name>` account on your laptop
  ```bash
    git clone https://github.com/RCHI-Lab/HAT2.git
    cd HAT2
    git sparse-checkout init
    git sparse-checkout set "hat_pkg/src/"
    ```
  If you use git version older than 2.25.0, please follow [this link](https://stackoverflow.com/questions/600079/how-do-i-clone-a-subdirectory-only-of-a-git-repository) to manually setup the sparse checkout.
* Under the `scripts` folder install the following requirements:
    ```bash
  pip3 install numpy
  pip3 install scipy
  pip3 install pickle
  pip3 install serial
  pip3 install time
  pip3 install pygame
  pip3 install pyttsx3
  ```
* Make the workspace
  ```bash
  catkin_make
  ```
* Add the robot and the laptop IP to the hosts file
  ```bash
  sudo nano /etc/hosts
  ```
  * Add the following lines:
  ```bash
  <robot_ip> robot
  <laptop_ip> laptop
  ```
* Update the `bashrc`
  ```bash
  nano ~/.bashrc
  ```
  * Add the following lines to the `bashrc`:
    ```bash
    source /home/<account_name>/catkin_ws/devel/setup.bash
    export ROS_MASTER_URI=http://laptop:11311
    export ROS_HOSTNAME=<laptop_ip> 
    ```
* Close the current terminal
* Source the updated `bashrc`
  ```bash
  source devel/setup.bash
  ```
* Ensure that the laptop is on the network
    ```bash
    ping laptop
    ```

