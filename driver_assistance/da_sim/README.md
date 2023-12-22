# da_sim

> This package is used for testing shared autonomy algorithms in simulation.

run gazebo simulation:

```shell
roslaunch stretch_gazebo gazebo.launch rviz:=true
```

run keyboard teleoperation:

```shell
roslaunch da_sim single_goal_ctrl.launch
# or
roslaunch da_sim multi_goal_ctrl.launch
```

run auto grasp:

```shell
roslaunch da_sim auto_reach.launch
```

run shared autonomy using keyboard interface:

```shell
roslaunch da_sim single_goal_ctrl.launch shared:=true
```

The default mode is set to **soft** constraint. Constraint type can be changed by modifying following line of `shared_controller.py` in `da_core` package.

```python
ctrler = AutoReachController(fixed_joints=(), constraint="soft")
```

multiple goal with intent recognition

```shell
roslaunch da_sim multi_goal_ctrl.launch shared:=true
```
