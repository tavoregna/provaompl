#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include "tf2_msgs/TFMessage.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Point.h"
#include <stdio.h>
#include <math.h>

char * getCostMap(int * wid,int * hei)
{
    nav_msgs::OccupancyGrid msg;
    msg=*(ros::topic::waitForMessage<nav_msgs::OccupancyGrid>("/map"));
    char * toRet;

    unsigned int width=msg.info.width;
    unsigned int height=msg.info.height;
    toRet=(char *)malloc(width*height);

    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {

            toRet[j+i*width]=msg.data[j+i*width];
        }
    }

    *wid=width;
    *hei=height;
    return toRet;
}

char * getCostMap2(int * wid,int * hei)
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
    toRet=(char *)malloc(width*height);

    for(int i=0;i<height;i++)
    {
        for(int j=0;j<width;j++)
        {

            toRet[j+i*width]=map.data[j+i*width];
            int val=0;
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
            printf("%-2d",val);

        }
        printf("\n");
    }

    *wid=width;
    *hei=height;
    return toRet;
}

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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "prova_mappa");

    int width;
    int height;
    getCostMap2(&width,&height);
    printf("width: %d \n",width);
    printf("height: %d \n",height);

    getRobotPose();

    return 0;
}
