#include "ros/ros.h"
#include <stdio.h>
#include <iostream>
#include <math.h>

#include "nav_msgs/GetMap.h"
#include "nav_msgs/OccupancyGrid.h"

#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Point.h"

#include "geometry_msgs/Twist.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#include "tf/tf.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
#include "tf/transform_datatypes.h"
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

#include <costmap_2d/costmap_2d_ros.h>


char * getCostMap(int * wid,int * hei,double * res,geometry_msgs::Pose * origin)
{
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<nav_msgs::GetMap>("static_map");

    nav_msgs::GetMap srv;

    if (!client.call(srv))
    {
        ROS_ERROR("Failed to call service static_map");
        return 0;
    }

    nav_msgs::OccupancyGrid map=srv.response.map;
    char * toRet;

    unsigned int width=map.info.width;
    unsigned int height=map.info.height;
    double resolution=map.info.resolution;
    geometry_msgs::Pose pose=map.info.origin;
    toRet=(char *)malloc(width*height);

    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {

            toRet[j+i*width]=map.data[j+i*width];
            /* int val=0;
            if(toRet[j+i*width]==0)
            {
                val=1;
                printf("\x1b[31m%-2d\x1b[0m",val);
            }
            else if(toRet[j+i*width]>0)
            {
                val=2;
                printf("\x1b[33m%-2d\x1b[0m",val);
            }
            else
            printf("%-2d",val);*/

        }
        //printf("\n");
    }

    *wid=width;
    *hei=height;
    *res=resolution;
    *origin=pose;
    return toRet;
}

//nel sistema di riferimento /map
void getRobotPose()
{
    geometry_msgs::PoseWithCovarianceStamped msg;

    msg=*(ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose"));

    geometry_msgs::Point position=msg.pose.pose.position;
    geometry_msgs::Quaternion orientation=msg.pose.pose.orientation;

    printf("position: \n");
    printf("x: %f \n",position.x);
    printf("y: %f \n",position.y);
    printf("z: %f \n",position.z);
    printf("orientation: \n");
    printf("x: %f \n",orientation.x);
    printf("y: %f \n",orientation.y);
    printf("z: %f \n",orientation.z);
    printf("w: %f \n",orientation.w);

    double yaw   =  asin(2*orientation.x*orientation.y + 2*orientation.z*orientation.w);

    printf("yaw: %f \n",yaw);
}

bool sendVelocityCommand(double x,double yaw)
{
    ros::NodeHandle nh;
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("turtlebot/mobile_base/commands/velocity", 1);
    //ros::Rate loop_rate(10000);
    geometry_msgs::Twist twist;
    twist.linear.x=0;
    twist.angular.z=0;
    cmd_pub.publish(twist);
ros::Rate loop_rate(10000);
    loop_rate.sleep();
    twist.linear.x=x;
    twist.angular.z=yaw;
   cmd_pub.publish(twist);
ros::Rate loop_rate1(10000);
    loop_rate1.sleep();
    cmd_pub.shutdown();
    nh.shutdown();
    return false;
}

bool sendGoalPose(double x,double y,double yaw)
{
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server");
    }

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    //goal.target_pose.header.seq = 100;

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;
    //goal.target_pose.pose.position.z = 0;

    goal.target_pose.pose.orientation.w = 1;
    //goal.target_pose.pose.orientation.x = 0;
    //goal.target_pose.pose.orientation.y =0;
    //goal.target_pose.pose.orientation.z = 0;

    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        return true;
    else{
        return false;
    }
}

geometry_msgs::PointStamped mapCoordinate_to_robotCoordinate(geometry_msgs::PointStamped map_point)
{
    geometry_msgs::PointStamped turtlebot_point;

    map_point.header.frame_id="map";
    turtlebot_point.header.frame_id="/turtlebot/base_footprint";

    tf::TransformListener listener;
    try
    {
        ros::Time now = ros::Time::now();
        //listener.waitForTransform("/map","/turtlebot/base_footprint",now,ros::Duration(3.0));

        listener.transformPoint("/turtlebot/base_footprint",map_point, turtlebot_point);
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    return turtlebot_point;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "programma");
    /*int width;
    int height;
    double resolution;
    geometry_msgs::Pose origin;
    getCostMap(&width,&height,&resolution,&origin);
    printf("width: %d\n",width);
    printf("height: %d\n",height);
    printf("resolution: %f\n",resolution);
    printf("ORIGIN: \n");
    printf("position: \n");
    printf("x: %f\n",origin.position.x);
    printf("y: %f\n",origin.position.y);
    printf("z: %f\n",origin.position.z);
    printf("orientation: \n");
    printf("x: %f\n",origin.orientation.x);
    printf("y: %f\n",origin.orientation.y);
    printf("z: %f\n",origin.orientation.z);
    printf("w: %f\n",origin.orientation.w);*/

  /*  tf::TransformListener tf(ros::Duration(10));
    costmap_2d::Costmap2DROS costmap("my_costmap", tf);
    tf::Stamped<tf::Pose> global_pose;
    geometry_msgs::PoseStamped pose;
    costmap.getRobotPose(global_pose);
    tf::poseStampedTFToMsg (global_pose, pose);
    getRobotPose();
    printf("\n\n");
    printf("position: \n");
    printf("x: %f\n",pose.pose.position.x);
    printf("y: %f\n",pose.pose.position.y);
    printf("z: %f\n",pose.pose.position.z);
    printf("orientation: \n");
    printf("x: %f\n",pose.pose.orientation.x);
    printf("y: %f\n",pose.pose.orientation.y);
    printf("z: %f\n",pose.pose.orientation.z);
    printf("w: %f\n",pose.pose.orientation.w);

std::cout << "\nGLOBAL FRAME ID: " << costmap.getGlobalFrameID();
printf("\n\nres: %f",costmap.getCostmap()->getResolution ());
for(int i=0;i<costmap.getCostmap()->getSizeInCellsX();i++)
{
for(int j=0;j<costmap.getCostmap()->getSizeInCellsY();j++)
{
costmap.getCostmap()->setCost(i,j,0);
}
}*/
/*if (ros::param::has("/move_base/global_costmap/inflation_layer/enabled"))
{
  printf("\nPARAMETTRO c'è");
ros::param::set("/move_base/global_costmap/inflation_layer/enabled",false);
std::string param;
bool d;
ros::param::get("/move_base/global_costmap/inflation_layer/enabled",d);
std::cout << "\nPARAM: " << d;
}
else
{
printf("\nPARAMETTRO NONNNNNNNNNNNNNNNNNNNNNNn c'è");
}

if (ros::param::has("/move_base/local_costmap/inflation_layer/enabled"))
{
  printf("\nPARAMETTRO c'è");
ros::param::set("/move_base/local_costmap/inflation_layer/enabled",false);
std::string param;
bool d;
ros::param::get("/move_base/local_costmap/inflation_layer/enabled",d);
std::cout << "\nPARAM: " << d;
}
else
{
printf("\nPARAMETTRO NONNNNNNNNNNNNNNNNNNNNNNn c'è");
}

if (ros::param::has("/move_base/global_costmap/obstacle_layer/enabled"))
{
  printf("\nPARAMETTRO c'è");
ros::param::set("/move_base/global_costmap/obstacle_layer/enabled",false);
std::string param;
bool d;
ros::param::get("/move_base/global_costmap/obstacle_layer/enabled",d);
std::cout << "\nPARAM: " << d;
}
else
{
printf("\nPARAMETTRO NONNNNNNNNNNNNNNNNNNNNNNn c'è");
}

if (ros::param::has("/move_base/local_costmap/obstacle_layer/enabled"))
{
  printf("\nPARAMETTRO c'è");
ros::param::set("/move_base/local_costmap/obstacle_layer/enabled",false);
std::string param;
bool d;
ros::param::get("/move_base/local_costmap/obstacle_layer/enabled",d);
std::cout << "\nPARAM: " << d;
}
else
{
printf("\nPARAMETTRO NONNNNNNNNNNNNNNNNNNNNNNn c'è");
}

if (ros::param::has("/move_base/local_costmap/robot_radius"))
{
  printf("\nPARAMETTRO c'è");
ros::param::set("/move_base/local_costmap/robot_radius",0.01);
std::string param;
double d;
ros::param::get("/move_base/local_costmap/robot_radius",d);
std::cout << "\nPARAM: " << d;
}
else
{
printf("\nPARAMETTRO NONNNNNNNNNNNNNNNNNNNNNNn c'è");
}

if (ros::param::has("/move_base/global_costmap/robot_radius"))
{
  printf("\nPARAMETTRO c'è");
ros::param::set("/move_base/global_costmap/robot_radius",0.01);
std::string param;
double d;
ros::param::get("/move_base/global_costmap/robot_radius",d);
std::cout << "\nPARAM: " << d;
}
else
{
printf("\nPARAMETTRO NONNNNNNNNNNNNNNNNNNNNNNn c'è");
}*/
dynamic_reconfigure::ReconfigureRequest srv_req;
dynamic_reconfigure::ReconfigureResponse srv_resp;
dynamic_reconfigure::DoubleParameter double_param;
dynamic_reconfigure::Config conf;

double_param.name = "/move_base/local_costmap/robot_radius";
double_param.value = 0.01;
conf.doubles.push_back(double_param);

/*double_param.name = "kurtana_roll_joint";
double_param.value = yaw;
conf.doubles.push_back(double_param);*/

srv_req.config = conf;

ros::service::call("/joint_commander/set_parameters", srv_req, srv_resp);


    return 0;
}

