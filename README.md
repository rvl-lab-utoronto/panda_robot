# RVL University of Toronto

This repository contains instructions for creating Gym like interfaces through ROS using the real Franka Emika Panda arm, for prototyping RL algorithms.

# Running the Panda Arm through ROS or OpenAI Gym 

We assume there are two Ubuntu 18.04 machines: one running real-time Linux (let's assume it's an Intel NUC), called NUC and connected directly to the Franka FCI computer via the network 192.168.131.x, and another (let's assume it's a GPU-enabled machine), called GPU and connected to the NUC via the network 192.168.132.x. We assume you have followed the instructions for setting up ROS communication between different machines. 

## Installation on the NUC machine
We assume ROS Melodic and libfranka 0.5, franka_ros have already been installed. Note: please install franka_ros from source, not as a binary through apt-get. Checkout the melodic-devel branch.
```
cd ~/catkin_ws/src
git clone https://github.com/rvl-lab-utoronto/panda_robot.git
cd ..
catkin_make
```

## Installation on the GPU machine
We assume ROS Melodic, libfranka 0.5, franka_ros, OpenAI gym, and the Kinect Azure drivers have already been installed. Note: please install franka_ros from source, not as a binary through apt-get. Checkout the melodic-devel branch.
```
cd ~/catkin_ws/src
git clone https://github.com/rvl-lab-utoronto/panda_robot.git
cd ..
catkin_make
```

## Live Joystick Control
The joystick can be connected either to the NUC or to the GPU machine. 

Then run the following on the NUC:
```
roslaunch panda_robot panda_robot.launch
```
and the following on the GPU (or NUC):
```
roslaunch panda_robot franka_arm_vel_joy.launch
```
Use the two axes of the joystick to control the end effector in cartesian velocity mode. Press the red button to set the arm to home configuration, using joint position control.


## Creating, Editing, and Installing OpenAI Gym Environments
We provide a sample reacher environment in `~/catkin_ws/src/panda_robot/src/gym-panda-robot/gym_panda_robot/envs/panda_reacher_env.py` In order to install it in your system and make it available to any RL algorithm, you need to run 
```
cd ~/catkin_ws/src/panda_robot/src/gym-panda-robot
sudo pip install --upgrade --force-reinstall -e .
```
You need to run this every time you update the reacher environment or you declare new environments in the envs directory.

## Running OpenAI Gym Environments

Run the following on the NUC:
```
roslaunch panda_robot panda_robot.launch
```
and the following on the GPU machine:
```
roslaunch panda_robot kinect_azure.launch
```
and
```
cd ~/catkin_ws/src/panda_robot/src
python panda_env_test.py
```
This simple test should make the robot move along the x,y and z axes in cartesian velocity mode, and occasionally
will reset to the home configuration.

## Contributors 

@florianshkurti @homangab @cheneyuwu
