## Lab7 **Autonomous Navigation with SMACH**

### Contents 

- [ArUco Marker Recognition and Its Use for Pose Estimation](https://github.com/SunstanYu/EE346_Line_Follower.git)
- [LiDAR-Based Navigation with TurtleBot3](https://github.com/SunstanYu/EE346_Lab6.git)
- [Autonomous Navigation with SMACH](https://github.com/SunstanYu/EE346_Lab7.git)



### Autonomous Navigation with SMACH

The objective is starting the robot from location P1, and the robot should successively move to P2, P3, P4 and back to P1 in that order before moving toward the ArUco marker to recognize the ID of the marker placed near the red triangle in the floor plan. The ArUco marker ID is a number n = 2, 3 or 4 and, upon recognizing the ID, the robot should beep with the buzzer on TurtleBot3 n times, and then move to Pn before coming to a stop.

![Image](https://github.com/SunstanYu/EE346_Lab7/raw/master/Image/image-20230114141830897.png)

### Tasks

#### Task 1: Point Navigation

The solution for this task is same as it in Lab6. Based on the LiDAR sensor, we build a grid map of the robot environment in the lab with GMapping and use amcl and move_base to instruct the navigation of robot. And in the scripts we defined the position of the points and publish the topic of move_base with manual order to realize the four points navigation.

#### Task 2: ArUco Detection

Base on the aruco_marker_finder.launch file, we can detect the ArUco marker of defined ArUco number. So we change the content of aruco_marker_finder.launch to make the detecting range from 2 to 4. Once the ArUco marker is detect, it will publish the topic of corresponding ID. Therefore, the job of detect ArUco ID is finished.

#### Task 3: Buzzer

Through sound_play library, we can play the specific sound by sending the relative topic. Therefore, the sound_play function is used in the smach state of detecting ArUco ID and play different sound when received different ArUco ID.



### Usage

1. Clone source code

   ```bash
   cd ~/catkin_ws/src
   git clone git@github.com:SunstanYu/EE346_Lab7.git
   ```

2. Connect and start the robot and robot's camera

   ```bash
   ssh pi@{IP_ADDRESS_OF_RASPBERRY_PI}
   roslaunch turtlebot3_bringup turtlebot3_robot.launch
   roslaunch raspicam_node camerav2_410x308_30fps.launch
   ```

3. Open the map generated in lab5

   ```bash
   roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
   ```

4. Run the arcuco finder to detect ArUco

   ```bash
   cd ~/catkin_ws/src/aruco-ros
   roslaunch aruco_marker_finder.launch
   ```

5. Run the script to perform the autonomous navigation

   ```bash
   cd ~/catkin_ws/src/EE346_Lab6/script
   python navigation.py
   ```



### Result

In the navigation task, the order of points is P1->P2->P3->P4->Detect Point.

![Image](https://github.com/SunstanYu/EE346_Lab7/raw/master/Image/navigation.gif)

In the detecting ArUco ID process, robot will stop in front of the ArUco marker and detect the number of ArUco marker.

![Image](https://github.com/SunstanYu/EE346_Lab7/raw/master/Image/detect.gif)
