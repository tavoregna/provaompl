#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

void wait(double sec)
{
ros::Rate rate(1.0/sec);
rate.sleep();
}
void moveRobot(double x,double yaw)
{
ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("/turtlebot/mobile_base/commands/velocity", 100);
  wait(2.0);
//int i=2;
//while(i>=0){
geometry_msgs::Twist msg0;
     msg0.linear.x = 0.0;     msg0.angular.z = 0.0;
     pub.publish(msg0);
wait(1.0);
     geometry_msgs::Twist msg;
     msg.linear.x = 0.2;     msg.angular.z = 0.0;
     pub.publish(msg);
wait(1.0);
for(int i=0;i<10;i++)
{
geometry_msgs::Twist msg1;
     msg1.linear.x = 0.0;     msg1.angular.z = 3.14/10.0;
     pub.publish(msg1);
printf("fatto %d\n",(i+1));
wait(1.0);
}

geometry_msgs::Twist msg2;
     msg2.linear.x = 0.2;     msg2.angular.z = 0.0;
     pub.publish(msg2); 
wait(1.0);
 

//i--;
 // } 
pub.shutdown();
nh.shutdown();
}

int main(int argc, char **argv){
  ros::init(argc, argv, "velocity_command");
//moveRobot(0.0,0.1);
//for(int i=0;i<10;i++)
  moveRobot(0.0,3.14/10.0);
//moveRobot(0.0,0.1);
}


