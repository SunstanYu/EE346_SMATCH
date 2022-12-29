#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>


/** function declarations **/

/** declare the coordinates of interest **/
double x1 = 2.204;
double y10 = -3.455;
double x2 = 0.769;
double y2 = 0.434;
double x3 = -3.07;
double y3 = -0.851;
double x4 = -1.61;
double y4 = -4.736;

bool goalReached = false;

bool moveToGoal(double xGoal, double yGoal){

   //define a client for to send goal requests to the move_base server through a SimpleActionClient
   actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

   //wait for the action server to come up
   while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
   }

   move_base_msgs::MoveBaseGoal goal;

   //set up the frame parameters
   goal.target_pose.header.frame_id = "base_link";
   goal.target_pose.header.stamp = ros::Time::now();

   /* moving towards the goal*/

   goal.target_pose.pose.position.x =  xGoal;
   goal.target_pose.pose.position.y =  yGoal;
   goal.target_pose.pose.position.z =  0.01;
   goal.target_pose.pose.orientation.x = 0.0;
   goal.target_pose.pose.orientation.y = 0.0;
   goal.target_pose.pose.orientation.z = 0.0;
   goal.target_pose.pose.orientation.w = 1.0;

   ROS_INFO("Sending goal location ...");
   ac.sendGoal(goal);

   ac.waitForResult();

   if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
      ROS_INFO("You have reached the destination");
      return true;
   }
   else{
      ROS_INFO("The robot failed to reach the destination");
      return false;
   }
}

int main(int argc, char** argv){
   ros::init(argc, argv, "simple_navigation_goals");
   ros::NodeHandle n;
   ros::spinOnce();

   int choice = 2;
   
   while(choice != 4){
      // if (choice == '0'){
      //    goalReached = moveToGoal(x1, y10);
      if (choice == 1){
        ROS_INFO("step 1!");
         goalReached = moveToGoal(x2-x1, y2-y10);
      }else if (choice == 2){
        ROS_INFO("step 2!");
         goalReached = moveToGoal(x3-x2, y3-y2);
      }else if (choice == 3){
        ROS_INFO("step 3!");
         goalReached = moveToGoal(x4-x3, y4-y3);
      }
      if (choice!=4){
         if (goalReached){
            ROS_INFO("Congratulations!");
            ros::spinOnce();
            // ros::spinOnce();
            choice++;
         }else{
            ROS_INFO("Hard Luck!");
         }
      }
   }
   return 0;
}



