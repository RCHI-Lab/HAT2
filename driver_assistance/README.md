# Driver Assistance for Teleoperation

[da_core](https://github.com/CalaW/driver_assistance/tree/master/da_core): core system blocks and algorithms for the driver assistance system

[da_interface](https://github.com/CalaW/driver_assistance/tree/master/da_interface): interface for the driver assistance system (keyboard, joystick and HAT)

[da_perception](https://github.com/CalaW/driver_assistance/tree/master/da_perception): perception modules for detecting objects

[da_sim](https://github.com/CalaW/driver_assistance/tree/master/da_sim): simulation environment for testing the driver assistance system

[da_stretch](https://github.com/CalaW/driver_assistance/tree/master/da_stretch): stretch specific modules for the driver assistance system

## Setup instructions

### Setting up hello robot related stuff

[Adding New Users - Stretch Documentation](https://docs.hello-robot.com/0.2/stretch-install/docs/add_new_user/)

[Robot Install - Stretch Documentation](https://docs.hello-robot.com/0.2/stretch-install/docs/robot_install/)

[Adding a ROS Workspace - Stretch Documentation](https://docs.hello-robot.com/0.2/stretch-install/docs/ros_workspace/)

Make the `~/stretch_user/log` directory if it saids `No such file or directory`

Once finished the setup process, you should have a `~/catkin_ws` workspace under your home folder.

source the workspace in your `~/.bashrc` by typing in the following command

```bash
source /home/<user_name>/catkin_ws/devel/setup.bash
```

Generate urdf:

```bash
roscd stretch_description/urdf
./xacro_to_urdf.sh
```

Install stretch_body

```bash
pip3 install -U hello-robot-stretch-body
pip3 install -U hello-robot-stretch-body-tools
```

Install numba

```bash
pip3 install numba
```

### Change stretch_ros to velocity control branch

```bash
cd ~/catkin_ws/src/stretch_ros
git switch feature/velocity_control
```

### Setup driver assistance

1. cd into `~/catkin_ws/src` and run:
    
    ```bash
    git clone https://github.com/RCHI-Lab/HAT2.git
    cd HAT2
    git sparse-checkout init
    git sparse-checkout set "driver_assistance/"
    ```
    If you use git version older than 2.25.0, please follow [this link](https://stackoverflow.com/questions/600079/how-do-i-clone-a-subdirectory-only-of-a-git-repository) to manually setup the sparce checkout.
    
2. Install pytorch, ref: https://pytorch.org/get-started/locally/
    
    ```bash
    pip3 install -U torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
    ```
    
    Install transformers, ref: https://huggingface.co/docs/transformers/installation
    
    ```bash
    pip3 install transformers
    ```
3. cd into `~/catkin_ws`, and run `rosdep install -i --from-path src --rosdistro $ROS_DISTRO`
4. Run `catkin_make` and source the workspace
5. Run perception system test once, it will download pretrained models
    
    ```bash
    roscd da_perception/scripts
    python3 owlvit_test.py
    ```
    
6. Test the detection

    ```bash
    roslaunch da_perception owl_vit.launch
    ```

## Perception Testing

Prompts are set in `driver_assistance/da_perception/config/queries.yaml` 

```yaml
queries:
  # - kleenex
  - tumbler
  - cup
  # - towel
```

To only run the perception testing

```bash
roslaunch da_perception owl_vit.launch
```

## Bring up the system

driver assistance:

```bash
roslaunch da_core driver_assistance.launch interface:=hat
```

teleoperation

```bash
roslaunch da_stretch teleop.launch interface:=hat
```
