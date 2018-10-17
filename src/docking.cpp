#define raggio 0.16
#define OMPL_time 10.0

//#define endX -2.05
//#define endY 0.74

#define endX 0.594540596008 //-1.27991485596
#define endY 1.49936294556 //-0.955833435059
#define yawY 0

#define MAX(X,Y) ((X) < (Y) ? (Y) : (X))
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/State.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/OptimizationObjective.h>
#include <ompl/base/samplers/ObstacleBasedValidStateSampler.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/PathControl.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <iostream>
#include <math.h>

#include <ompl/geometric/planners/sbl/pSBL.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>

#include <time.h>
#include "ros/ros.h"
#include <stdio.h>

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
#include "tf/transform_listener.h"

#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>




namespace ob = ompl::base;
namespace oc = ompl::control;


class OMPL_Planning{
    int width;
    int height;
    double cellDim;
    char * costMap;

    OMPL_Planning(int wid,int hei,double ced,char * cm)
    {
        width=wid;
        height=hei;
        cellDim=ced;
        costMap=cm;
    }

};









int width;
int height;
double resolution;
geometry_msgs::Pose origin;
char * map;

double startX;
double startY;
double startYaw;

double offsetX;
double offsetY;

int pathLength;
double * pathX;
double * pathY;



//ompl::geometric::PathGeometric ppp;

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

    for(int i=0;i<height*width;i++)
    {

        toRet[i]=map.data[i];
    }

    *wid=width;
    *hei=height;
    *res=resolution;
    *origin=pose;
    return toRet;
}

void toEulerAngle(geometry_msgs::Quaternion q, double* roll, double* pitch, double* yaw)
{
    // roll (x-axis rotation)
    double sinr = +2.0 * (q.w * q.x + q.y * q.z);
    double cosr = +1.0 - 2.0 * (q.x * q.x + q.y * q.y);
    *roll = atan2(sinr, cosr);

    // pitch (y-axis rotation)
    double sinp = +2.0 * (q.w * q.y - q.z * q.x);
    if (fabs(sinp) >= 1)
        *pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        *pitch = asin(sinp);

    // yaw (z-axis rotation)
    double siny = +2.0 * (q.w * q.z + q.x * q.y);
    double cosy = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    *yaw = atan2(siny, cosy);
}

void toQuaternion(double pitch, double roll, double yaw,geometry_msgs::Quaternion * q)
{

        // Abbreviations for the various angular functions
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);

        q->w = cy * cr * cp + sy * sr * sp;
        q->x = cy * sr * cp - sy * cr * sp;
        q->y = cy * cr * sp + sy * sr * cp;
        q->z = sy * cr * cp - cy * sr * sp;
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
    printf("QUATERNION: \n");
    printf("x: %f \n",orientation.x);
    printf("y: %f \n",orientation.y);
    printf("z: %f \n",orientation.z);
    printf("w: %f \n",orientation.w);
    printf("ROLL,PITCH,YAW: \n");

    double roll,pitch,yaw;
    toEulerAngle(orientation, &roll, &pitch, &yaw);
    printf("roll: %f \n",roll);
    printf("pitch: %f \n",pitch);
    printf("yaw: %f \n",yaw);



    double yaw2   =  asin(2*orientation.x*orientation.y + 2*orientation.z*orientation.w);

    printf("yaw2: %f \n",yaw2);

    startX=position.x;
    startY=position.y;
    startYaw=yaw;
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

    geometry_msgs::Quaternion q;
    toQuaternion(0.0,0.0,yaw,&q);


    goal.target_pose.pose.orientation.w = q.w;
    goal.target_pose.pose.orientation.x = q.x;
    goal.target_pose.pose.orientation.y = q.y;
    goal.target_pose.pose.orientation.z = q.z;

    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        return true;
    else{
        return false;
    }
}

void changeParameter(char name[],bool val)
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::BoolParameter enable_param;
  dynamic_reconfigure::Config conf;

  enable_param.name = "enabled";
  enable_param.value = val;
  conf.bools.push_back(enable_param);
srv_req.config = conf;
ros::service::call(name, srv_req, srv_resp);
}

void changeClearingRotation(bool val)
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::BoolParameter enable_param;
  dynamic_reconfigure::Config conf;

  enable_param.name = "clearing_rotation_allowed";
  enable_param.value = val;
  conf.bools.push_back(enable_param);
srv_req.config = conf;
ros::service::call("/move_base/set_parameters", srv_req, srv_resp);
}

void changeRecoveryBehavior(bool val)
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::BoolParameter enable_param;
  dynamic_reconfigure::Config conf;

  enable_param.name = "recovery_behavior_enabled";
  enable_param.value = val;
  conf.bools.push_back(enable_param);
srv_req.config = conf;
ros::service::call("/move_base/set_parameters", srv_req, srv_resp);
}

void changePrunePlan(bool val)
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::BoolParameter enable_param;
  dynamic_reconfigure::Config conf;

  enable_param.name = "prune_plan";
  enable_param.value = val;
  conf.bools.push_back(enable_param);
srv_req.config = conf;
ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);
}

void changeForwardPointDistance(double tol)
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter enable_param;
  dynamic_reconfigure::Config conf;

  enable_param.name = "forward_point_distance";
  enable_param.value = tol;
  conf.doubles.push_back(enable_param);
srv_req.config = conf;
ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);
}

void changePathDistanceBias(double tol)
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter enable_param;
  dynamic_reconfigure::Config conf;

  enable_param.name = "path_distance_bias";
  enable_param.value = tol;
  conf.doubles.push_back(enable_param);
srv_req.config = conf;
ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);
}

void changeGoalDistanceBias(double tol)
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter enable_param;
  dynamic_reconfigure::Config conf;

  enable_param.name = "goal_distance_bias";
  enable_param.value = tol;
  conf.doubles.push_back(enable_param);
srv_req.config = conf;
ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);
}

void changeOccdistScale(double tol)
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter enable_param;
  dynamic_reconfigure::Config conf;

  enable_param.name = "occdist_scale";
  enable_param.value = tol;
  conf.doubles.push_back(enable_param);
srv_req.config = conf;
ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);
}

void changeInflationLayerRadius(double tol)
{
  dynamic_reconfigure::ReconfigureRequest srv_req1;
  dynamic_reconfigure::ReconfigureResponse srv_resp1;
  dynamic_reconfigure::DoubleParameter enable_param1;
  dynamic_reconfigure::Config conf1;

  enable_param1.name = "inflation_radius";
  enable_param1.value = tol;
  conf1.doubles.push_back(enable_param1);
srv_req1.config = conf1;
ros::service::call("/move_base/local_costmap/inflation_layer/set_parameters", srv_req1, srv_resp1);

dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter enable_param;
  dynamic_reconfigure::Config conf;

  enable_param.name = "inflation_radius";
  enable_param.value = tol;
  conf.doubles.push_back(enable_param);
srv_req.config = conf;
ros::service::call("/move_base/global_costmap/inflation_layer/set_parameters", srv_req, srv_resp);
}

void changeInflationLayerCost(double tol)
{
  dynamic_reconfigure::ReconfigureRequest srv_req1;
  dynamic_reconfigure::ReconfigureResponse srv_resp1;
  dynamic_reconfigure::DoubleParameter enable_param1;
  dynamic_reconfigure::Config conf1;

  enable_param1.name = "cost_scaling_factor";
  enable_param1.value = tol;
  conf1.doubles.push_back(enable_param1);
srv_req1.config = conf1;
ros::service::call("/move_base/local_costmap/inflation_layer/set_parameters", srv_req1, srv_resp1);

dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter enable_param;
  dynamic_reconfigure::Config conf;

  enable_param.name = "cost_scaling_factor";
  enable_param.value = tol;
  conf.doubles.push_back(enable_param);
srv_req.config = conf;
ros::service::call("/move_base/global_costmap/inflation_layer/set_parameters", srv_req, srv_resp);
}

void changeYawGoalTolerance(double tol)
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter enable_param;
  dynamic_reconfigure::Config conf;

  enable_param.name = "yaw_goal_tolerance";
  enable_param.value = tol;
  conf.doubles.push_back(enable_param);
srv_req.config = conf;
ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);
}

void changeCostmapResolution(double tol)
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter enable_param;
  dynamic_reconfigure::Config conf;

  enable_param.name = "resolution";
  enable_param.value = tol;
  conf.doubles.push_back(enable_param);
srv_req.config = conf;
ros::service::call("/move_base/local_costmap/set_parameters", srv_req, srv_resp);
}

void changeXYGoalTolerance(double tol)
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter enable_param;
  dynamic_reconfigure::Config conf;

  enable_param.name = "xy_goal_tolerance";
  enable_param.value = tol;
  conf.doubles.push_back(enable_param);
srv_req.config = conf;
ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);
}

void changeSimulationTime(double tol)
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter enable_param;
  dynamic_reconfigure::Config conf;

  enable_param.name = "sim_time";
  enable_param.value = tol;
  conf.doubles.push_back(enable_param);
srv_req.config = conf;
ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);
}

void changeRotVel(double min,double max,double stop)
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter enable_param;
  dynamic_reconfigure::Config conf;

  enable_param.name = "min_rot_vel";
  enable_param.value = min;
  conf.doubles.push_back(enable_param);
  srv_req.config = conf;
ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);

dynamic_reconfigure::ReconfigureRequest srv_req1;
  dynamic_reconfigure::ReconfigureResponse srv_resp1;
  dynamic_reconfigure::DoubleParameter enable_param1;
  dynamic_reconfigure::Config conf1;

  enable_param1.name = "max_rot_vel";
  enable_param1.value = max;
  conf1.doubles.push_back(enable_param1);
srv_req1.config = conf1;
ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req1, srv_resp1);

dynamic_reconfigure::ReconfigureRequest srv_req2;
  dynamic_reconfigure::ReconfigureResponse srv_resp2;
  dynamic_reconfigure::DoubleParameter enable_param2;
  dynamic_reconfigure::Config conf2;

  enable_param2.name = "rot_stopped_vel";
  enable_param2.value = stop;
  conf2.doubles.push_back(enable_param2);
srv_req2.config = conf2;
ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req2, srv_resp2);
}


void changeTransVel(double min,double max,double stop)
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter enable_param;
  dynamic_reconfigure::Config conf;

  enable_param.name = "min_trans_vel";
  enable_param.value = min;
  conf.doubles.push_back(enable_param);
  srv_req.config = conf;
ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);

dynamic_reconfigure::ReconfigureRequest srv_req1;
  dynamic_reconfigure::ReconfigureResponse srv_resp1;
  dynamic_reconfigure::DoubleParameter enable_param1;
  dynamic_reconfigure::Config conf1;

  enable_param1.name = "max_trans_vel";
  enable_param1.value = max;
  conf1.doubles.push_back(enable_param1);
srv_req1.config = conf1;
ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req1, srv_resp1);

dynamic_reconfigure::ReconfigureRequest srv_req2;
  dynamic_reconfigure::ReconfigureResponse srv_resp2;
  dynamic_reconfigure::DoubleParameter enable_param2;
  dynamic_reconfigure::Config conf2;

  enable_param2.name = "trans_stopped_vel";
  enable_param2.value = stop;
  conf2.doubles.push_back(enable_param2);
srv_req2.config = conf2;
ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req2, srv_resp2);
}

void changeXVel(double min,double max)
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter enable_param;
  dynamic_reconfigure::Config conf;

  enable_param.name = "min_vel_x";
  enable_param.value = min;
  conf.doubles.push_back(enable_param);
  srv_req.config = conf;
ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);

dynamic_reconfigure::ReconfigureRequest srv_req1;
  dynamic_reconfigure::ReconfigureResponse srv_resp1;
  dynamic_reconfigure::DoubleParameter enable_param1;
  dynamic_reconfigure::Config conf1;

  enable_param1.name = "max_vel_x";
  enable_param1.value = max;
  conf1.doubles.push_back(enable_param1);
srv_req1.config = conf1;
ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req1, srv_resp1);
}


void changeRobotRadius(double res)
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter enable_param;
  dynamic_reconfigure::Config conf;

  enable_param.name = "robot_radius";
  enable_param.value = res;
  conf.doubles.push_back(enable_param);
srv_req.config = conf;
ros::service::call("/move_base/global_costmap/set_parameters", srv_req, srv_resp);


dynamic_reconfigure::ReconfigureRequest srv_req1;
  dynamic_reconfigure::ReconfigureResponse srv_resp1;
  dynamic_reconfigure::DoubleParameter enable_param1;
  dynamic_reconfigure::Config conf1;

  enable_param1.name = "robot_radius";
  enable_param1.value = res;
  conf1.doubles.push_back(enable_param1);
srv_req1.config = conf1;
ros::service::call("/move_base/local_costmap/set_parameters", srv_req1, srv_resp1);
}

void changeLocalMapStaticMap(bool val)
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::BoolParameter enable_param;
  dynamic_reconfigure::Config conf;

  enable_param.name = "static_map";
  enable_param.value = val;
  conf.bools.push_back(enable_param);
srv_req.config = conf;
ros::service::call("/move_base/local_costmap/set_parameters", srv_req, srv_resp);
}

void changeVXSamples(int val)
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::IntParameter enable_param;
  dynamic_reconfigure::Config conf;

  enable_param.name = "vx_samples";
  enable_param.value = val;
  conf.ints.push_back(enable_param);
srv_req.config = conf;
ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);
}

void changeVTHSamples(int val)
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::IntParameter enable_param;
  dynamic_reconfigure::Config conf;

  enable_param.name = "vth_samples";
  enable_param.value = val;
  conf.ints.push_back(enable_param);
srv_req.config = conf;
ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);
}

void changeLocalCostmapHeightWidth(int val)
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::IntParameter enable_param;
  dynamic_reconfigure::Config conf;

  enable_param.name = "height";
  enable_param.value = val;
  conf.ints.push_back(enable_param);
srv_req.config = conf;
ros::service::call("/move_base/local_costmap/set_parameters", srv_req, srv_resp);


dynamic_reconfigure::ReconfigureRequest srv_req1;
  dynamic_reconfigure::ReconfigureResponse srv_resp1;
  dynamic_reconfigure::IntParameter enable_param1;
  dynamic_reconfigure::Config conf1;

  enable_param1.name = "width";
  enable_param1.value = val;
  conf1.ints.push_back(enable_param1);
srv_req1.config = conf1;
ros::service::call("/move_base/local_costmap/set_parameters", srv_req1, srv_resp1);
}

void setInflationObstacleLayer(bool val)
{
changeParameter("/move_base/global_costmap/inflation_layer/set_parameters",val);
changeParameter("/move_base/local_costmap/inflation_layer/set_parameters",val);
changeParameter("/move_base/global_costmap/obstacle_layer/set_parameters",val);
changeParameter("/move_base/local_costmap/obstacle_layer/set_parameters",val);
changeParameter("/move_base/global_costmap/static_layer/set_parameters",val);
changeParameter("/move_base/local_costmap/static_layer/set_parameters",val);
}
/*void changeParameters(bool val)
{
changeParameter("/move_base/global_costmap/inflation_layer/set_parameters",val);
changeParameter("/move_base/local_costmap/inflation_layer/set_parameters",val);
changeParameter("/move_base/global_costmap/obstacle_layer/set_parameters",val);
changeParameter("/move_base/local_costmap/obstacle_layer/set_parameters",val);

changeYawGoalTolerance(3.14);
changeXYGoalTolerance(0.05);

changePathDistanceBias(100.0);

changeCostmapResolution(0.01);

changeInflationLayerRadius(0.001);
changeLocalCostmapHeightWidth(1);

changeClearingRotation(false);
changeRecoveryBehavior(false);

//changeInflationLayerCost(100.0);
}*/

void changeParameters1()
{	  
    changeRotVel(0.1,4.0,0.1);
    changeTransVel(0.1,0.5,0.1);
    changeXVel(0.0,0.5);

    changeSimulationTime(3.0);
    changeVTHSamples(30);
    changeVXSamples(10);

    changeRobotRadius(0.18);

    //changePrunePlan(true);

    changeInflationLayerCost(0.9);
    changeInflationLayerRadius(0.4);	
    changeCostmapResolution(0.05);

    setInflationObstacleLayer(true);

    changeYawGoalTolerance(0.7);
    changeXYGoalTolerance(0.10);

    changePathDistanceBias(2);
    changeGoalDistanceBias(60);
    changeOccdistScale(0.5);
   // changeForwardPointDistance(0.32);

    changeLocalCostmapHeightWidth(4);

    changeClearingRotation(true);
    changeRecoveryBehavior(true);
    //changeClearingRotation(false);
    //changeRecoveryBehavior(false);
}

void changeParameters2()
{
    changeRotVel(0.1,4.0,0.1);
    changeTransVel(0.1,0.3,0.1);
    changeXVel(0.0,0.5);

    changeSimulationTime(2.0);
    changeVTHSamples(20);
    changeVXSamples(10);

    changeRobotRadius(0.18);

    //changePrunePlan(true);

    changeInflationLayerCost(1.0);
    changeInflationLayerRadius(0.01);	
    changeCostmapResolution(0.05);

    setInflationObstacleLayer(true);

    changeYawGoalTolerance(3.14);
    changeXYGoalTolerance(0.03);

    changePathDistanceBias(50);
    changeGoalDistanceBias(25);
    changeOccdistScale(0.01);
    //changeForwardPointDistance(0.03);

    changeLocalCostmapHeightWidth(2);

    changeClearingRotation(false);
    changeRecoveryBehavior(false);
}

void changeParameters3()
{
    changeRotVel(0.1,4.0,0.1);
    changeTransVel(0.1,0.5,0.1);
    changeXVel(0.0,0.5);

    changeSimulationTime(3.0);
    changeVTHSamples(30);
    changeVXSamples(10);

    changeRobotRadius(0.18);

    //changePrunePlan(true);

    changeInflationLayerCost(0.9);
    changeInflationLayerRadius(0.4);	
    changeCostmapResolution(0.05);

    setInflationObstacleLayer(true);

    changeYawGoalTolerance(0.3);
    changeXYGoalTolerance(0.10);

    changePathDistanceBias(2);
    changeGoalDistanceBias(60);
    changeOccdistScale(0.5);
    //changeForwardPointDistance(0.32);

    changeLocalCostmapHeightWidth(4);

    changeClearingRotation(true);
    changeRecoveryBehavior(true);
    //changeClearingRotation(false);
    //changeRecoveryBehavior(false);
}

void changeParameters4()
{
    changeRotVel(0.1,4.0,0.1);
    changeTransVel(0.1,0.5,0.1);
    changeXVel(0.0,0.5);

    changeSimulationTime(1);
    changeVTHSamples(20);
    changeVXSamples(10);

    changeRobotRadius(0.18);

    //changePrunePlan(true);

    changeInflationLayerCost(100.0);
    changeInflationLayerRadius(0.001);	
    changeCostmapResolution(0.05);

    setInflationObstacleLayer(false);
    changeYawGoalTolerance(0.3);
    changeXYGoalTolerance(0.05);

    changePathDistanceBias(32);
    changeGoalDistanceBias(20);
    changeOccdistScale(0.5);
    //changeForwardPointDistance(0.20);

    changeLocalCostmapHeightWidth(4);

    changeClearingRotation(false);
    changeRecoveryBehavior(false);
}

void changeParameters5()
{
    changeRotVel(0.1,1.0,0.1);
    changeTransVel(0.1,0.3,0.1);
    changeXVel(0.0,0.5);

    changeSimulationTime(2.0);
    changeVTHSamples(20);
    changeVXSamples(10);

    changeRobotRadius(0.18);

    //changePrunePlan(true);

    changeInflationLayerCost(100.0);
    changeInflationLayerRadius(0.001);	
    changeCostmapResolution(0.05);

    setInflationObstacleLayer(true);

    changeYawGoalTolerance(3.14);
    changeXYGoalTolerance(0.03);

    changePathDistanceBias(50);
    changeGoalDistanceBias(25);
    changeOccdistScale(0.01);
    //changeForwardPointDistance(0.03);

    changeLocalCostmapHeightWidth(1);

    changeClearingRotation(false);
    changeRecoveryBehavior(false);
}



bool isStateValid(const oc::SpaceInformation *si, const ob::State *state)
{
    const auto *se2state = state->as<ob::SE2StateSpace::StateType>();

    const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);
    const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1);

    double x=(*pos).values[0];
    double y=(*pos).values[1];
    double yaw=(*rot).value;

    double dimCella=resolution;


    if(x<=-offsetX+raggio || y<=-offsetY+raggio || x>=offsetX-raggio || y>=offsetY-raggio)
        return false;

    x=x+offsetX;
    y=y+offsetY;

    double minX=x-raggio;
    double maxX=x+raggio;
    double minY=y-raggio;
    double maxY=y+raggio;

    

    int indexMinX=(int)floor(minX/dimCella);
    int indexMaxX=(int)floor(maxX/dimCella);
    int indexMinY=(int)floor(minY/dimCella);
    int indexMaxY=(int)floor(maxY/dimCella);

    // double costMap[300][300];
    // &(&costMap)=(double **)calloc(300*300,sizeof(double));

    /*  for(int i=0;i<300;i++)
    {
        for(int j=0;j<300;j++)
        {
            costMap[i][j]=0;
        }
    }*/

    /*    for(int j=0;j<height;j++)
    {
        for(int i=0;i<width;i++)
        {
            char v=map[i+j*width];
            if((i>=indexMinX && i<=indexMaxX) && (j>=indexMinY && j<=indexMaxY))
                printf("\x1b[36m%s\x1b[0m","@");
            else if(v==-1)
                printf("\x1b[30m%-d\x1b[0m",0);
            else if(v==0)
                printf("\x1b[32m%-d\x1b[0m",1);
            else
                printf("\x1b[33m%-d\x1b[0m",2);

        }
        printf("\n");
    }*/

    for(int i=indexMinX;i<=indexMaxX;i++)
    {
        for(int j=indexMinY;j<=indexMaxY;j++)
        {
            if(map[i+j*width]!=0)//costMap[j][i]!=0)
            {
                return false;
            }
        }
    }
    return true;
}

void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result)
{
    const auto *se2state = start->as<ob::SE2StateSpace::StateType>();
    const double* pos = se2state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    const double rot = se2state->as<ob::SO2StateSpace::StateType>(1)->value;
    const double* ctrl = control->as<oc::RealVectorControlSpace::ControlType>()->values;

    result->as<ob::SE2StateSpace::StateType>()->setXY(
                pos[0] + ctrl[0] * duration * cos(rot),
            pos[1] + ctrl[0] * duration * sin(rot));
    result->as<ob::SE2StateSpace::StateType>()->setYaw(
                rot    + ctrl[1] * duration);
}

ob::ValidStateSamplerPtr allocOBValidStateSampler(const ob::SpaceInformation *si)
{
    // we can perform any additional setup / configuration of a sampler here,
    // but there is nothing to tweak in case of the ObstacleBasedValidStateSampler.
    return std::make_shared<ob::ObstacleBasedValidStateSampler>(si);
}

ob::OptimizationObjectivePtr getPathLengthObjective(const ob::SpaceInformationPtr& si)
{
    return ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si));
}

void plan()
{

    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE2StateSpace>());

    // set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    /*bounds.setLow(0,-offsetX);
    bounds.setHigh(0,offsetX);
    bounds.setLow(1,-offsetY);
    bounds.setHigh(1,offsetY);*/
    bounds.setLow(0,MIN(startX,endX)-(raggio*2));
    bounds.setHigh(0,MAX(startX,endX)+(raggio*2));
    bounds.setLow(1,MIN(startY,endY)-(raggio*2));
    bounds.setHigh(1,MAX(startY,endY)+(raggio*2));

    space->setBounds(bounds);

    space->setLongestValidSegmentFraction(0.001);

    // create a control space
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));

    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    cbounds.setLow(0,-0.3);
    cbounds.setHigh(0,0.3);
    cbounds.setLow(1,-0.3);
    cbounds.setHigh(1,0.3);

    cspace->setBounds(cbounds);

    // construct an instance of  space information from this control space
    auto si(std::make_shared<oc::SpaceInformation>(space, cspace));

    // set state validity checking for this space
    si->setStateValidityChecker(
                [&si](const ob::State *state) { return isStateValid(si.get(), state); });
    
    si->setStateValidityCheckingResolution(0.001);

    // set the state propagation routine
    si->setStatePropagator(propagate);

    si->setValidStateSamplerAllocator(allocOBValidStateSampler);

    // create a start state
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(startX);
    start->setY(startY);
    start->setYaw(startYaw);

    // create a goal state
    ob::ScopedState<ob::SE2StateSpace> goal(start);
    /*(*goal)[0]->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0.71;
    (*goal)[0]->as<ob::RealVectorStateSpace::StateType>()->values[1] = 2.0;
    (*goal)[1]->as<ob::SO2StateSpace::StateType>()->value = 0.0;*/
    (*goal)[0]->as<ob::RealVectorStateSpace::StateType>()->values[0] = endX;
    (*goal)[0]->as<ob::RealVectorStateSpace::StateType>()->values[1] = endY;
    (*goal)[1]->as<ob::SO2StateSpace::StateType>()->value = 0.0;



    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal, 0.001);

    pdef->setOptimizationObjective(getPathLengthObjective(si));

    // create a planner for the defined space

    //auto planner(std::make_shared<ompl::geometric::pSBL>(si));
    //auto planner(std::make_shared<ompl::geometric::RRTConnect>(si));
    //auto planner(std::make_shared<ompl::geometric::RRTstar>(si));
    //auto planner(std::make_shared<ompl::geometric::BKPIECE1>(si));
    //auto planner(std::make_shared<oc::EST>(si));
    //auto planner(std::make_shared<oc::KPIECE1>(si));
    //auto planner(std::make_shared<ompl::geometric::SPARStwo>(si));
    auto planner(std::make_shared<ompl::geometric::InformedRRTstar>(si));


    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
    planner->setup();


    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    // attempt to solve the problem within ten seconds of planning time
    ob::PlannerStatus solved = planner->ob::Planner::solve(OMPL_time);

    ob::PathPtr pathh;
pathLength=-1;

    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        pathh = pdef->getSolutionPath();
        std::cout << "Found solution:" << std::endl;

        // print the path to screen
        pathh->print(std::cout);
    }
    else
    {
        std::cout << "No solution found" << std::endl;
        return;
    }

    ompl::geometric::PathGeometric path=*(pathh->as<ompl::geometric::PathGeometric>());
    path.interpolate(20);
    pathLength=path.getStateCount();
    pathX=(double *)malloc(pathLength*sizeof(double));
    pathY=(double *)malloc(pathLength*sizeof(double));

    for(int i=0;i<path.getStateCount();i++)
    {
        const auto *se2state = (path.getState(i))->as<ob::SE2StateSpace::StateType>();

        const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);
        const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1);

        double x=(*pos).values[0];
        double y=(*pos).values[1];

	pathX[i]=x;
        pathY[i]=y;

        printf("%d: X: %d, Y: %d\n",i+1,(int)floor((x+offsetX)/resolution),(int)floor((y+offsetY)/resolution));
        map[((int)floor((x+offsetX)/resolution))+((int)floor((y+offsetY)/resolution))*width]=3;


    }

    for(int j=0;j<height;j++)
    {
        for(int i=0;i<width;i++)
        {
            char v=map[i+j*width];

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
}

int main(int argc, char ** argv)
{

    int iSecret, iGuess;

      /* initialize random seed: */
    //  srand (time(NULL));

      /* generate secret number between 1 and 10: */
     /// iSecret = rand() % 100000 + 1;
    ros::init(argc, argv, "dockiiiiing");
    double roll,pitch,yaw;
    geometry_msgs::Quaternion q;
    q.x=0.0;
    q.y=0.0;
    q.z=0.00983377994223; //-0.998006783775;
    q.w=0.999951647217; //0.0631067313312;
    toEulerAngle(q, &roll, &pitch, &yaw);
    while(!sendGoalPose(endX,endY,yaw));
    return 0;
}
