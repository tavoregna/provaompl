#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "tf2_msgs/TFMessage.h"
#include "nav_msgs/GetMap.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include <stdio.h>
#include <math.h>
#include "tf/tf.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Point.h"

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
void provaConversione()
{
    // tf::Stamped<tf::Point> stamped_in();
    // tf::Stamped<tf::Point> stamped_out();
    // tf::Transformer::transformPoint("turtlebot/base_footprint",&stamped_in,&stamped_out);
    geometry_msgs::PointStamped turtlebot_point;
    geometry_msgs::PointStamped map_point;




    map_point.header.frame_id="map";
    map_point.point.x = -1,629843;
    map_point.point.y = -0,358113;
    map_point.point.z=-1,602598;

    printf("input point:\n");
    printf("x: %f\n",map_point.point.x);
    printf("y: %f\n",map_point.point.y);
    printf("z: %f\n\n",map_point.point.z);

    tf::Stamped<tf::Point> tf_point_stamped_in, tf_point_stamped_out;
    tf::pointStampedMsgToTF(map_point,tf_point_stamped_in);

  //  tf::Transformer transformer;

   // transformer.transformPoint("/turtlebot/base_link", tf_point_stamped_in, tf_point_stamped_out);

   // tf::pointStampedTFToMsg(tf_point_stamped_out,turtlebot_point);

    tf::TransformListener tl;
    tl.transformPoint("base_link",map_point,turtlebot_point);

    tf::pointStampedTFToMsg(tf_point_stamped_out,turtlebot_point);

    printf("output point:\n");
    printf("x: %f\n",turtlebot_point.point.x);
    printf("y: %f\n",turtlebot_point.point.y);
    printf("z: %f\n",turtlebot_point.point.z);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "prova_mappa");

    int width;
    int height;
    getCostMap(&width,&height);
    printf("width: %d \n",width);
    printf("height: %d \n",height);

    getRobotPose();

   // provaConversione();

    return 0;
}
