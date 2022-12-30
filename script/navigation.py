import rospy
import actionlib
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from random import sample
from math import pow, sqrt
import smach
import smach_ros
import sys
from sound_play.msg import SoundRequest
from sound_play.libsoundplay import SoundClient

class Nav(smach.State):
    def __init__(self):
        self.currentseq2 = 0;
        self.currentseq3 = 0;
        self.currentseq4 = 0;

        smach.State.__init__(self,outcomes=['outcome1'])  
        global cmd 
        cmd = AutoNav()
        cmd.set_initial_pose1()
        print(1)
       
    def execute(self, userdata):
        # cmd.move_goal(0)
        print(5)
        cmd.move_goal(1)
        cmd.move_goal(2)
        cmd.move_goal(3)
        cmd.move_goal(4)
        return 'outcome1'
    

class Aruco(smach.State):
    def __init__(self):
        self.currentseq2 = 10000;
        self.currentseq3 = 10000;
        self.currentseq4 = 10000;
        print(3)

        smach.State.__init__(self,outcomes=['outcome2','outcome3','outcome4'])
        rospy.Subscriber("/aruco_2/pose", PoseStamped, self.positionchange2, queue_size=10)
        rospy.Subscriber("/aruco_3/pose", PoseStamped, self.positionchange3, queue_size=10)
        rospy.Subscriber("/aruco_4/pose", PoseStamped, self.positionchange4, queue_size=10)        
        

    def execute(self, userdata):
        print(4)
        while not rospy.is_shutdown():
            if self.currentseq2<10000:
                # rospy.init_node('play', anonymous=True)
                soundhandle = SoundClient()
                rospy.sleep(1)
                rospy.loginfo('Playing sound 2.')
                soundhandle.play(4, 3)
                rospy.sleep(1)
                return 'outcome2'
            elif self.currentseq3<10000:
                # rospy.init_node('play', anonymous=True)
                soundhandle = SoundClient()
                rospy.sleep(1)
                rospy.loginfo('Playing sound 3.')
                soundhandle.play(3, 3)
                rospy.sleep(1)
                return 'outcome3'
            elif self.currentseq4<10000:
                # rospy.init_node('play', anonymous=True)
                soundhandle = SoundClient()
                rospy.sleep(1)
                rospy.loginfo('Playing sound 4.')
                soundhandle.play(2, 3)
                rospy.sleep(1)
                return 'outcome4'       
    
    def positionchange2(self, msg):
        rospy.loginfo("Label 2 detected!")
        self.currentseq2 = msg.header.seq
        distance = msg.pose.position.z
        rospy.loginfo("distance"+str(distance))

    def positionchange3(self, msg):
        rospy.loginfo("Label 3 detected!")
        self.currentseq3 = msg.header.seq
        distance = msg.pose.position.z
        rospy.loginfo("distance"+str(distance))
    
    def positionchange4(self, msg):
        rospy.loginfo("Label 4 detected!")
        self.currentseq4 = msg.header.seq
        distance = msg.pose.position.z
        rospy.loginfo("distance"+str(distance))

class NavSingle1(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['outcome5'])

    def execute(self, userdata):
        global cmd
        cmd.set_initial_pose2()
        cmd.move_goal(1)
        return 'outcome5'

class NavSingle2(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['outcome5'])
    def execute(self, userdata):
        global cmd
        cmd.set_initial_pose2()
        cmd.move_goal(2)
        return 'outcome5'

class NavSingle3(smach.State):
    def __init__(self):
        smach.State.__init__(self,outcomes=['outcome5'])
    def execute(self, userdata):
        global cmd
        cmd.set_initial_pose2()
        cmd.move_goal(3)
        return 'outcome5'        

goalPoints = [ 
    # from point1 to point2, to point3, to point4 and then back to point1
    # position[XYZ] and pose[quaternion]
    # In our map of lab, X-direction is from bottom to top and Y-direction is from right to left
    [(2.159, -3.395, 0.000), (0.000, 0.000, 0.8328, 0.5534)],  #pose of point1 index:0 
    [(0.769, 0.434, 0.000), (0.000, 0.000, -0.670, 0.743)],    #pose of point2 index:1 
    [(-3.07, -0.8516, 0.000), (0.000, 0.000, 0.786, 0.618)],   #pose of point3 index:2 Point(-3.07, -0.8516, 0.000), Quaternion(0.000, 0.000, 0.786, 0.618)
    [(-1.61, -4.73, 0.000), (0.000, 0.000, 0.733, 0.680)],  #pose of point4 index:3 Point(-1.61, -4.73, 0.000), Quaternion(0.000, 0.000, 0.733, 0.680)
    [(1.623, -3.189, 0.0), (0.0, 0.0, 0.8425, 0.5386)],  #pose of start point   index:5
    [(1.565, -1.583, 0.0), (0.0, 0.0, 0.8180, 0.5751)]  #test point   index:6

]

class AutoNav:
    def __init__(self):
        print(0)
        self.moveBaseAction = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.init_pose_pub = rospy.Publisher("initialpose", PoseWithCovarianceStamped, latch=True, queue_size=1)


        wait_status = self.moveBaseAction.wait_for_server(rospy.Duration(10))
        rospy.loginfo("Waiting for move_base action server...")
        if not wait_status:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        
        rospy.loginfo("Connected to move base server!")
        rospy.on_shutdown(self.shutdown_hook)

    def shutdown_hook(self):
        rospy.loginfo("Stopping the robot...")
        # Cancel any active goals
        self.moveBaseAction.cancel_goal()
        rospy.sleep(2)
        # Stop the robot
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)
        self.cmd_vel_pub.publish(Twist())
        rospy.sleep(1)

    def set_initial_pose1(self):
        '''To set the 2D pose estimate of initial moment'''
        init_pose = PoseWithCovarianceStamped()
        init_pose.header.frame_id = 'map'
        init_pose.pose.pose.position.x = 2.159
        init_pose.pose.pose.position.y = -3.395
        init_pose.pose.pose.position.z = 0.000
        init_pose.pose.pose.orientation.x = 0.000
        init_pose.pose.pose.orientation.y = 0.000
        init_pose.pose.pose.orientation.z = 0.8425
        init_pose.pose.pose.orientation.w = 0.5534
        init_pose.pose.covariance[0] = 0.25
        init_pose.pose.covariance[7] = 0.25
        init_pose.pose.covariance[35] =  0.06853892326654787
        self.init_pose_pub.publish(init_pose)

    def set_initial_pose2(self):
        '''To set the 2D pose estimate of initial moment'''
        init_pose = PoseWithCovarianceStamped()
        init_pose.header.frame_id = 'map'
        init_pose.pose.pose.position.x = 1.623         # 1.565  
        init_pose.pose.pose.position.y = -3.189        #-1.583 
        init_pose.pose.pose.position.z = 0.0
        init_pose.pose.pose.orientation.x = 0.0
        init_pose.pose.pose.orientation.y = 0.0
        init_pose.pose.pose.orientation.z = 0.8425  # 0.8180 
        init_pose.pose.pose.orientation.w = 0.5386   #0.5751
        init_pose.pose.covariance[0] = 0.25
        init_pose.pose.covariance[7] = 0.25
        init_pose.pose.covariance[35] =  0.06853892326654787
        self.init_pose_pub.publish(init_pose)    

    def set_goal(self, index):
        '''
        To produce and return the goal pose variable which contains position and orientation
        '''
        goal_pose = MoveBaseGoal()
        pose = goalPoints[index]
        goal_pose.target_pose.header.frame_id = 'map'
        goal_pose.target_pose.pose.position.x = pose[0][0]
        goal_pose.target_pose.pose.position.y = pose[0][1]
        goal_pose.target_pose.pose.position.z = pose[0][2]
        goal_pose.target_pose.pose.orientation.x = pose[1][0]
        goal_pose.target_pose.pose.orientation.y = pose[1][1]
        goal_pose.target_pose.pose.orientation.z = pose[1][2]
        goal_pose.target_pose.pose.orientation.w = pose[1][3]
        return goal_pose

    def move_goal(self, index, wait_time=60):
        '''To send the move command and wait for the result'''
        goal = self.set_goal(index)
        self.moveBaseAction.send_goal(goal)
        # wait_time = 60   #unit:seconds
        rospy.loginfo("Begin to navigate autonomous to the point"+str(index+1))
        finish_status = self.moveBaseAction.wait_for_result(rospy.Duration(wait_time))
        if not finish_status:
            self.moveBaseAction.cancel_goal()
            rospy.loginfo(str(wait_time)+" seconds time out without reaching the goal")
        else:
            if self.moveBaseAction.get_state() == GoalStatus.SUCCEEDED:
                rospy.loginfo("Arrive the point "+str(index+1))


#     def __init__(self):
        
#         rospy.on_shutdown(self.shutdown)
        
#         # How long in seconds should the robot pause at each location?
#         self.rest_time = rospy.get_param("~rest_time", 1)
        
#         # Are we running in the fake simulator?
#         self.fake_test = rospy.get_param("~fake_test", False)
        
#         # Goal state return values
#         goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
#                        'SUCCEEDED', 'ABORTED', 'REJECTED',
#                        'PREEMPTING', 'RECALLING', 'RECALLED',
#                        'LOST']
        
#         # Set up the goal locations. Poses are defined in the map frame.  
#         # An easy way to find the pose coordinates is to point-and-click
#         # Nav Goals in RViz when running in the simulator.
#         # Pose coordinates are then displayed in the terminal
#         # that was used to launch RViz.
#         locations = dict()
        
#         locations['point1'] = Pose(Point(2.159, -3.395, 0.000), Quaternion(0.000, 0.000, 0.9517, 0.3069))
#         locations['point2'] = Pose(Point(0.769, 0.434, 0.000), Quaternion(0.000, 0.000, -0.670, 0.743))
#         locations['point3'] = Pose(Point(-3.07, -0.8516, 0.000), Quaternion(0.000, 0.000, 0.786, 0.618))
#         locations['point4'] = Pose(Point(-1.61, -4.73, 0.000), Quaternion(0.000, 0.000, 0.733, 0.680))

#         # locations['living_room_2'] = Pose(Point(1.471, 1.007, 0.000), Quaternion(0.000, 0.000, 0.480, 0.877))
#         # locations['dining_room_1'] = Pose(Point(-0.861, -0.019, 0.000), Quaternion(0.000, 0.000, 0.892, -0.451))
        
#         # Publisher to manually control the robot (e.g. to stop it, queue_size=5)
#         self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        
#         # Subscribe to the move_base action server
#         self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        
#         rospy.loginfo("Waiting for move_base action server...")
        
#         # Wait 60 seconds for the action server to become available
#         self.move_base.wait_for_server(rospy.Duration(60))
        
#         rospy.loginfo("Connected to move base server")
        
#         # A variable to hold the initial pose of the robot to be set by 
#         # the user in RViz
#         initial_pose = PoseWithCovarianceStamped()
        
#         # Variables to keep track of success rate, running time,
#         # and distance traveled
#         n_locations = len(locations)
#         n_goals = 0
#         n_successes = 0
#         i = n_locations
#         distance_traveled = 0
#         start_time = rospy.Time.now()
#         running_time = 0
#         location = ""
#         last_location = ""
#         i=0
#         last_i=i
#         # Get the initial pose from the user
#         rospy.loginfo("*** Click the 2D Pose Estimate button in RViz to set the robot's initial pose...")
#         rospy.wait_for_message('initialpose', PoseWithCovarianceStamped)
#         self.last_location = Pose()
#         rospy.Subscriber('initialpose', PoseWithCovarianceStamped, self.update_initial_pose)
        
#         # Make sure we have the initial pose
#         while initial_pose.header.stamp == "":
#             rospy.sleep(1)
            
#         rospy.loginfo("Starting navigation test")
        
#         # Begin the main loop and run through a sequence of locations
#         while not rospy.is_shutdown():
#             # If we've gone through the current sequence,
#             # start with a new random sequence
#             # if i == n_locations:
#             #     i = 0
#             #     sequence = sample(locations, n_locations)
#             #     # Skip over first location if it is the same as
#             #     # the last location
#             #     if sequence[0] == last_location:
#             #         i = 1
            
#             sequence = list(locations.keys());
#             if i>=len(sequence):
#                 break
#             location=sequence[i]
#             # Get the next location in the current sequence
#             # Keep track of the distance traveled.
#             # Use updated initial pose if available.
#             if initial_pose.header.stamp == "":
#                 distance = sqrt(pow(locations[location].position.x - 
#                                     locations[last_location].position.x, 2) +
#                                 pow(locations[location].position.y - 
#                                     locations[last_location].position.y, 2))
#             else:
#                 rospy.loginfo("Updating current pose.")
#                 distance = sqrt(pow(locations[location].position.x - 
#                                     initial_pose.pose.pose.position.x, 2) +
#                                 pow(locations[location].position.y - 
#                                     initial_pose.pose.pose.position.y, 2))
#                 initial_pose.header.stamp = ""
            
#             # Store the last location for distance calculations
#             last_location = location
#             last_i=i
#             # Increment the counters
#             i += 1
#             n_goals += 1
        
#             # Set up the next goal location
#             self.goal = MoveBaseGoal()
#             self.goal.target_pose.pose = locations[location]
#             self.goal.target_pose.header.frame_id = 'map'
#             self.goal.target_pose.header.stamp = rospy.Time.now()
            
#             # Let the user know where the robot is going next
#             rospy.loginfo("Going to: " + str(location))
            
#             # Start the robot toward the next location
#             self.move_base.send_goal(self.goal)
            
#             # Allow 5 minutes to get there
#             finished_within_time = self.move_base.wait_for_result(rospy.Duration(300)) 
            
#             # Check for success or failure
#             if not finished_within_time:
#                 self.move_base.cancel_goal()
#                 rospy.loginfo("Timed out achieving goal")
#             else:
#                 state = self.move_base.get_state()
#                 if state == GoalStatus.SUCCEEDED:
#                     rospy.loginfo("Goal succeeded!")
#                     n_successes += 1
#                     distance_traveled += distance
#                     rospy.loginfo("State:" + str(state))
#                 elif state == GoalStatus.ABORTED:
#                     rospy.loginfo("Goal aborted")
#                     i=last_i
#                     n_goals=i
#                 else:
#                   rospy.loginfo("Goal failed with error code: " + str(goal_states[state]))
            
#             # How long have we been running?
#             running_time = rospy.Time.now() - start_time
#             running_time = running_time.secs / 60.0
            
#             # Print a summary success/failure, distance traveled and time elapsed
#             rospy.loginfo("Success so far: " + str(n_successes) + "/" + 
#                           str(n_goals) + " = " + 
#                           str(100 * n_successes/n_goals) + "%")
#             rospy.loginfo("Running time: " + str(trunc(running_time, 1)) + 
#                           " min Distance: " + str(trunc(distance_traveled, 1)) + " m")
#             rospy.sleep(self.rest_time)
            
#     def update_initial_pose(self, initial_pose):
#         self.initial_pose = initial_pose

#     def shutdown(self):
#         global mode
#         mode=1
#         rospy.loginfo("Stopping the robot...")
#         self.move_base.cancel_goal()
#         rospy.sleep(2)
#         self.cmd_vel_pub.publish(Twist())
#         rospy.sleep(1)


class park():   
    def __init__(self):
        rospy.init_node('park')
        global lastseq
        global currentseq
        global position
        global direction
        global distance
        distance=10
        direction=0
        lastseq = 0
        currentseq=0
        velocityx=0
        anglez=0
        self.pub=rospy.Publisher("/cmd_vel",Twist, queue_size=5)
        rospy.Subscriber("/aruco_4/pose", PoseStamped, self.positionchange, queue_size=1)

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Starting parking")
        rate=rospy.Rate(20.0)
        arucofind=0
        while not rospy.is_shutdown():
            
            if distance > 0.4:
               arucofind=1 
               if currentseq!=lastseq:
                 lastseq=currentseq
                 if direction == 0:
                    velocityx=0.08
                    anglez=-0.1
                 elif direction == 1:
                    velocityx=0.08
                    anglez=0.3
                 else:
                    velocityx=0.1
                    anglez=0
                    
               else:
                    velocityx=0.08
                    anglez=0
            elif distance>0.1:
                arucofind=1
                if currentseq!=lastseq:
                 lastseq=currentseq
                 if direction == 0:
                    velocityx=0.05
                    anglez=-0.2
                 elif direction == 1:
                    velocityx=0.05
                    anglez=0.2
                 else:
                    velocityx=0.05
                    anglez=0
                elif arucofind:
                    velocityx=0
                    anglez=0                     
                else:
                    velocityx=0.05
                    anglez=0               
            else:
                velocityx=0
                anglez=0
                print(distance)
            vel_msg=Twist()
            vel_msg.linear.x=velocityx
            vel_msg.angular.z=anglez
            self.pub.publish(vel_msg)
            rospy.loginfo("parking")
            
            rate.sleep()

    def positionchange(self, msg):
        global direction
        global distance
        global currentseq 
        global position 

        currentseq = msg.header.seq
        position = msg.pose.position
        x=msg.pose.position.x
        rospy.loginfo("pose x"+str(x))
        if x>0.05:
            #right
            direction=0
        elif x<0:
            #left
            direction=1
        else:
            direction=2
        distance = msg.pose.position.z
        rospy.loginfo("distance"+str(distance))

    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        rospy.sleep(2)
        self.pub.publish(Twist())
        rospy.sleep(1)

if __name__ == '__main__':
    try:
        rospy.init_node("navController_class")
        global mode 
        # a=park()
        # rospy.spin()
        sm_top = smach.StateMachine(outcomes=['result'])
    
    # Open the container
        with sm_top:

          smach.StateMachine.add('Nav', Nav(),
                               transitions={'outcome1':'Aruco'})
          print(2)                     
          smach.StateMachine.add('Aruco', Aruco(), 
                                   transitions={'outcome2':'Nav2',
                                                'outcome3':'Nav3',
                                                'outcome4':'Nav4'})
          print(4)                                      
        # Create the sub SMACH state machine

            # Add states to the container
          smach.StateMachine.add('Nav2', NavSingle1(), 
                                   transitions={'outcome5':'result'})

          smach.StateMachine.add('Nav3', NavSingle2(), 
                                   transitions={'outcome5':'result'})
            
          smach.StateMachine.add('Nav4', NavSingle3(), 
                                   transitions={'outcome5':'result'})

        sm_top.execute()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("AMCL navigation test finished.")
