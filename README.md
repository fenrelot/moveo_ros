# This is a clone of jesseweisberg/moveo_ros but will be 6DOF version (maybe if i get it up and running) 

Found some improvements for the Moveo (endstops, some extra bearings and a 6th DOF ("rotating hand")) on thingiverse ! https://www.thingiverse.com/thing:2146252 by labala

should run on 1x arduino mega 2560 with ramps 1.4 for all motors. 
Changes to the original code are:

Stepper Z is now for Joint 6 !!!
There is an Arduino CNC shield attached to RAMPS AUX2 port, for the two Steppers which control the Joint 2 (nema23)

using ros Kinetic ! (because code was made for kinetic...if everything is running i will try to port to Neotic (moveit visual tools not yet running on neotic ?))

# Installation of stuff
1. install Ubuntu 16.04 amd64 on ssd
2. install ROS like in http://wiki.ros.org/kinetic/Installation/Ubuntu
	install fill desktop version (sudo apt-get install ros-kinetic-desktop-full)
	install catkin !!! and make a workspace which works
3. install moveit https://moveit.ros.org/install/ with  (sudo apt-get install ros-kinetic-moveit) and install also moveit visual tools https://github.com/ros-planning/moveit_visual_tools via ( sudo apt-get install ros-kinetic-moveit-visual-tools )

4. follow the instructions from jesse weisberg (original description further down)

moveit rviz should work! then we need rosserial !!!

# Update
everything seems to run at the moment, and my robot is working (no bearings yet...but everything moves)
rostopic pub joint_steps moveo_moveit/ArmJointState <Joint1 Joint2 Joint3 Joint4 Joint5 0> (needs to be put like rostopic pub joint_steps moveo_moveit/ArmJointState 100 100 100 100 100 0 )
	- Change "Joint1, Joint2, etc." to the number of steps you want each joint to move.
	
this worked! but value 0 seems to be the starting value for all joints, and thats not cool because putting negative values seems to not work!... soo what can we do instead? 

fortunately the code of jesse all in all seems to have been written actually for 6 axis robot...which is nice.

. Limits in the moveit config seem to be very much off (foind that from the moveit config tutorial). also im using different microstepping settings...and im gona use a planetary gearbox 1:5 for the joint 4 (direct drive is really not working well)

(also i found that controlling the endeffector with xbox360 controller is very laggy but cpu is idling when starting joystick node so there seems to be other issues with that) 

next up will be updating the urdf files with the 6th axis

# UPdate 2
so there is a urdf file generator for solidworks !!! ...how convenient...i guess everybody used that.
...but that program is way to expensive for this project.... so fortunately i found https://github.com/andresaraque/centauri6dof <- some guys who also used the 6 axis mod of the moveo and exported the kinematic model. they also have some gui which is fun to play with (already tested and seems to work)

...next up planetary gearbox für joint 4 and playing around with the code.... will also try if this centauri6dof code runs on newer ROS versions // 16.04 is tooo old


//Original Text
# moveo_ros
ROS packages that can be used to plan and execute motion trajectories for the BCN3D Moveo robotic arm in simulation and real-life.
### [Video Demo Here!](https://youtu.be/2RcTTqs17O8)

- **_New Feature_: Object-Specific Pick and Place** (With an ordinary webcam, Tensorflow, OpenCV, and ROS, you can 'pick and place' (or sort) objects that are detected in real-time)
	- **[Video Demo](https://youtu.be/kkUbyFa2MWc)**
	- **[How to Use](https://github.com/jesseweisberg/moveo_ros/tree/master/moveo_moveit/scripts)**



## How to Use:

### Getting the BCN3D Simulation Working with Motion Planning
![moveit_screenshot.png](/moveit_screenshot.png)

1. Make sure you have ROS installed correctly with a functioning workspace-- I used ROS Kinetic on Ubuntu 16.04 (if you have a different distro, you may need to change some things).  I currently have 'moveo_ros' in the 'src' folder of my catkin workspace.

2. To plan and execute trajectories for the Moveo in simulation (RVIZ with Moveit plugin), execute the following terminal command:
	```
	roslaunch moveo_moveit_config demo.launch
	```

3. Once the window loads, in the bottom-left corner check "Allow Approximate IK Solutions."  Then click on the "Planning" tab in the MotionPlanning panel of RVIZ.  Select a new goal state by either dragging the interactive marker (light blue ball on the end effector) or under "Select Goal State."  Once goal state is updated, "Plan and Execute" will plan and execute the trajectory from the start state to the updated goal state.


### Moving the real robot, synced with the simulated robot's trajectories.
4. Make sure you download the AccelStepper ([AccelStepper Library Download](http://www.airspayce.com/mikem/arduino/AccelStepper/AccelStepper-1.57.zip)) and ros_lib ([rosserial-arduino tutorial](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)) libraries into your Arduino environment.
	- If ros_lib already exists in your Arduino libraries (<Arduino sketchbook>/libraries), follow the last troubleshooting tip or you'll get an error saying "ArmJointState.h: no such file".  ROS makes you remove ros_lib and regenerate it every time you introduce a new custom message.

5. Change the pin layout between your robot and the RAMPS 1.4 in **'moveo_moveit_arduino.ino'** and upload the file to your Arduino (I'm using MEGA 2560).  Make sure the robot and the simulation are in the same position (to set the simulation upright initially-- select "Upright" from "Select Goal States" in RVIZ.

6. In 'moveit_convert.cpp' replace the stepsPerRevolution array with the steps/revolution (or microsteps/revolution) of each of your motors.  (Note: if you don't already know these values, you can experimentally get how many microsteps/revolution your motors have using the MultiStepperTest.ino and recording/eyeballing the results)

7. With the simulation already running, execute each of the following commands in it's own, separate terminal: 
	- ``` rosrun rosserial_python serial_node.py /dev/ttyUSB0 ```(establishes rosserial node that communicates with Arduino)
	- ```rosrun moveo_moveit moveit_convert ``` (converts simulation joint_state rotations to steps and publishes on the /joint_steps topic, which the Arduino script subscribes to)
	- ```rostopic pub gripper_angle std_msgs/UInt16 <angle 0-180> ```(publishes gripper_angle)

**Now, whatever trajectories are planned and executed in simulation are echoed on the real robot.**

## About Directories
### moveo_urdf
Contains the URDF (Unified Robot Description File) for the BCN3D Moveo. Necessary for simulation in RVIZ and moveit configuration.

### moveo_moveit_config
Configuration for moveit, a motion planning framework that has a plugin in RVIZ, which is what we are using here.

### moveo_moveit
- _moveit_convert.cpp_: Converts simulation 'joint\_state' rotations (from the 'move\_group/fake\_controller\_joint\_states' topic) to steps and publishes on the /joint\_steps topic.  Joint\_steps is an array of 6 Int16 values (though we only have 5 joints in this case) that represent the accumulated steps executed by each joint since the moveit\_convert node has started running. 

- _move\_group\_interface\_coor\_1.cpp_: Can hardcode a pose/position for the end effector in the script and plan/execute a trajectory there.  Also reads/outputs the current pose/position of the end effector.

## Troubleshooting
- After step 7, there should be 3 new topics created: 
	- **/joint\_steps**: steps necessary to move each motor to desired position
	- **/joint\_steps\_feedback**: same as /joint_steps, except published back by arduino to check that information is being received by Arduino correctly 
	- **/gripper\_angle**: current angle of the gripper

- **To move Moveo from the command line:**
	- ```rostopic pub joint_steps moveo_moveit/ArmJointState <Joint1 Joint2 Joint3 Joint4 Joint5 0>```  
	- Change "Joint1, Joint2, etc." to the number of steps you want each joint to move.

- **Use ```rostopic list``` and search for these topics to check if they are currently running**

- **Use ```rostopic echo /<topic>``` to view the data on \<topic> in your terminal** 

- If you get the following ```"error: moveo_moveit/ArmJointState.h: No such file or directory"```, perform the following steps in terminal:
	```
	cd <Arduino sketchbook>/libraries
	rm -rf ros_lib 
	rosrun rosserial_arduino make_libraries.py .
	```
	- More info on the ROS wiki: 
		- Section 2.2 here: (http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup)
		- (http://wiki.ros.org/rosserial/Tutorials/Adding%20Other%20Messages)
	
- Here is my current layout and wiring schematic for reference:
![aerialRobotSketch.pdf](/aerial_robot_sketch.png)
