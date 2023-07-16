run gazebo simulation:

```shell
roslaunch stretch_gazebo gazebo.launch rviz:=true
```

run keyboard teleoperation:

```shell
roslaunch driver_assistance sim_single_goal_ctrl.launch
# or
roslaunch driver_assistance sim_multi_goal_ctrl.launch
```

run auto grasp:

```shell
roslaunch driver_assistance sim_auto_reach.launch
```

run shared autonomy using keyboard interface:

```shell
roslaunch driver_assistance sim_single_goal_ctrl.launch shared:=true
```

The default mode is set to **soft** constraint. Constraint type can be changed by modifying following line of `shared_controller.py`.

```python
ctrler = AutoReachController(fixed_joints=(), constraint="soft")
```

multiple goal with intent recognition

```shell
roslaunch driver_assistance sim_multi_goal_ctrl.launch shared:=true
```
