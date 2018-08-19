#include "ros/ros.h"

#include "nav_msgs/OccupancyGrid.h"

//#include <tf/transform_listener.h>
//#include <costmap_2d/costmap_2d_ros.h>


void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    //ROS_INFO("I heard: ");
    ROS_INFO("I heard: [%d]",msg->info.width);

    // ROS_INFO("I heard: [%s]", (msg->header).frame_id);
    // sub.shutdown();
    unsigned int width=msg->info.width;
    unsigned int height=msg->info.height;

    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {
            ROS_INFO("%d",msg->data[j+i*width]);
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "prova_mappa");

    ros::NodeHandle nh;
    ros::Subscriber sub;

    //   sub = nh.subscribe("map", 1000, mapCallback);

    nav_msgs::OccupancyGrid msg;
    msg=*(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map"));

    unsigned int width=msg.info.width;
    unsigned int height=msg.info.height;

    char costMap[height][width];
    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {

            costMap[i][j]=msg.data[j+i*width];
            ROS_INFO("%d",costMap[i][j]);
        }
    }
    ROS_INFO("FINE!");

    return 0;
}
