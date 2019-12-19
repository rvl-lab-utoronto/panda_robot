# Panda Arm Instructions


## Running things individually

See the README inside Panda/

## Build Workspace
```
catkin build or catkin_make build
chmod +x src/panda_arm/src/remap_joy_to_twist.py
```

## Running on nuc
For joystick control
```
rosrun panda_arm remap_joy_to_twist.py
```

```
roslaunch panda_arm franka_arm_vel_joy.launch robot_ip:=192.168.131.40 load_gripper:=false
```

Note: Position controller is not tested.
```
roslaunch panda_arm franka_arm_pos_joy.launch robot_ip:=192.168.131.40 load_gripper:=false
```

For hooking up the controller and the `panda_env`:

```
roslaunch franka_example_controllers cartesian_velocity_example_controller.launch robot_ip:=192.168.131.40 load_gripper:=false

```
```
roslaunch panda_arm panda_env.launch
```

## Running from basestation (pc with gpu)
Make sure ROS communication is working as expected between the `nuc` and the pc (instructions provided inside the main `README.md` on the previous page.) If this is working, you should be able to see `franka_control` and `state` etc topics.
Inside the following lunch files, make sure the `ip` address of the `nuc` is set correctly. 
```
franka_control_state_publisher.launch & panda_moveit.launch
```

Inside `panda_env.py`, also make sure the path to the launch files is set correctly. The global variable `PANDA_REAL`. This should be changed in the future to be the path of the package, as opposed to a hard-coded path.

You can then run the `launch` files as above.
