#include "ros/ros.h"

#include "nav_msgs/OccupancyGrid.h"

#include <iostream>
#include <fstream>


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

for(int j=0;j<height;j++)
    {
        for(int i=0;i<width;i++)
        {
            char v=msg.data[i+j*width];

            if(v==-1)
                printf("\x1b[30m%-d\x1b[0m",0);
            else if(v==0)
                printf("\x1b[32m%-d\x1b[0m",1);
            else if(v==3)
                printf("\x1b[46m\x1b[36m%-s\x1b[0m","@");
            else
                printf("\x1b[33m%-d\x1b[0m",2);

        }
        printf("\n");

    }
    return 0;
}
