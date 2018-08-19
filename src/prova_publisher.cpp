// ROS Default Header File
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "move_base_msgs/MoveBaseActionGoal.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
// MsgTutorial Message File Header
// The header file is automatically created when building the package.
//#include "provaompl/MsgTutorial.h"

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char **argv)
// Node Main Function
{
    ros::init(argc, argv, "prova_publisher");	// Initializes Node Name
    ros::NodeHandle nh;
    // Node handle declaration for communication with ROS system
    // Declare publisher, create publisher 'ros_tutorial_pub' using the 'MsgTutorial'
    // message file from the 'provaompl' package. The topic name is
    // 'ros_tutorial_msg' and the size of the publisher queue is set to 100.
    ros::Publisher ros_tutorial_pub1 =
            nh.advertise<geometry_msgs::Twist>("/turtlebot/mobile_base/commands/velocity", 100);
    // Set the loop period. '10' refers to 10 Hz and the main loop repeats at 0.1 second intervals
    ros::Rate loop_rate(10);
    geometry_msgs::Twist msg;

    int count = 0;
    // Declares message 'msg' in 'MsgTutorial' message
    // file format
    // Variable to be used in message
    // while (ros::ok())
    while(ros::ok() && count<10)
    {
        //msg.stamp = ros::Time::now();	 // Save current time in the stamp of 'msg'
        // msg.data
        //      = count;
        msg.linear.x=0;
        msg.linear.y=0;
        msg.linear.z=0;

        msg.angular.x=1;
        msg.angular.y=1;
        msg.angular.z=1;

        // Save the the 'count' value in the data of 'msg'
        //    ROS_INFO("send msg = %d", msg.stamp.sec);
        //   ROS_INFO("send msg = %d", msg.stamp.nsec);	 // Print the 'stamp.nsec' message
        //  ROS_INFO("send msg = %d", msg.data);		 // Print the 'data' message
        ros_tutorial_pub1.publish(msg);		 // Publishes 'msg' message
        loop_rate.sleep();			 // Goes to sleep according to the loop rate defined above.
        ++count;					 // Increase count variable by one
    }
    ROS_INFO("FINITA LA PRIMA PARTE");

    /* ros::Publisher ros_tutorial_pub2 =
            nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base_simple/goal", 100);
    // Set the loop period. '10' refers to 10 Hz and the main loop repeats at 0.1 second intervals
    move_base_msgs::MoveBaseActionGoal msg1;

    count = 0;
    // Declares message 'msg' in 'MsgTutorial' message
    // file format
    // Variable to be used in message
    // while (ros::ok())
    if(ros::ok())
    {
        //msg.stamp = ros::Time::now();	 // Save current time in the stamp of 'msg'
        // msg.data
        //      = count;
        msg1.header.seq=0;
        msg1.header.stamp=ros::Time::now();
        msg1.header.frame_id="0";

        msg1.goal_id.stamp=ros::Time::now();
        msg1.goal_id.id="0";


        msg1.goal.target_pose.header.seq=0;
        msg1.goal.target_pose.header.stamp=ros::Time::now();
        msg1.goal.target_pose.header.frame_id="0";

        msg1.goal.target_pose.pose.position.x=1.0;
        msg1.goal.target_pose.pose.position.y=1.0;
        msg1.goal.target_pose.pose.position.z=0.0;

        msg1.goal.target_pose.pose.orientation.x=1.0;
        msg1.goal.target_pose.pose.orientation.y=1.0;
        msg1.goal.target_pose.pose.orientation.z=0.0;
        msg1.goal.target_pose.pose.orientation.w=0.0;


        // Save the the 'count' value in the data of 'msg'
        //    ROS_INFO("send msg = %d", msg.stamp.sec);
        //   ROS_INFO("send msg = %d", msg.stamp.nsec);	 // Print the 'stamp.nsec' message
        //  ROS_INFO("send msg = %d", msg.data);		 // Print the 'data' message
        ros_tutorial_pub2.publish(msg1);		 // Publishes 'msg' message
        loop_rate.sleep();			 // Goes to sleep according to the loop rate defined above.
        ++count;					 // Increase count variable by one
    }*/

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server  qwertytrew");
    }

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = -1.675609;
    goal.target_pose.pose.position.y = -2.117073;
    goal.target_pose.pose.orientation.w = 0.115678;

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("You have arrived to the goal position");
    else{
        ROS_INFO("The base failed for some reason");
    }

    return 0;
}
