#define raggio 0.21
#define OMPL_time 4.0
#define stampaMappa false

//#define endX -2.05
//#define endY 0.74



#define numeroTentativi 3

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
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/PathControl.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/config.h>
#include <iostream>
#include <math.h>

#include <ompl/geometric/planners/sbl/pSBL.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/prm/SPARStwo.h>
#include <ompl/geometric/planners/kpiece/BKPIECE1.h>
#include <ompl/geometric/planners/rrt/RRTsharp.h>



#include "ros/ros.h"
#include <stdio.h>
#include <termios.h>

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
#include <dynamic_reconfigure/StrParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>

#include <fstream>
#include <string>
#include <stdlib.h>
#include <map>
#include <thread>

using namespace std;


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
char * mappa;

double offsetX;
double offsetY;

int pathLength;
double * pathX;
double * pathY;

//ompl::geometric::PathGeometric ppp;

char * getCostMap(int * wid,int * hei,double * res,geometry_msgs::Pose * origin)
{
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<nav_msgs::GetMap>("dynamic_map"); //static map

    nav_msgs::GetMap srv;

    if (!client.call(srv))
    {
        ROS_ERROR("Failed to call service static_map");
        return 0;
    }

    nav_msgs::OccupancyGrid mapgrid=srv.response.map;
    char * toRet;

    unsigned int width=mapgrid.info.width;
    unsigned int height=mapgrid.info.height;
    double resolution=mapgrid.info.resolution;
    geometry_msgs::Pose pose=mapgrid.info.origin;
    toRet=(char *)malloc(width*height);

    for(int i=0;i<height*width;i++)
    {

        toRet[i]=mapgrid.data[i];
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

double getYawFromQuaternion(double xx,double yy,double zz,double ww)
{
    geometry_msgs::Quaternion q;
    q.x=xx;
    q.y=yy;
    q.z=zz;
    q.w=ww;
    double roll;
    double pitch;
    double yaw;
    toEulerAngle(q,&roll,&pitch,&yaw);
    return yaw;
}

//nel sistema di riferimento /map
void getRobotPose(double* x, double* y, double* yaw)
{
    geometry_msgs::PoseWithCovarianceStamped msg;

    msg=*(ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>("poseupdate")); //"amcl_pose"

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

    double roll,pitch,yaww;
    toEulerAngle(orientation, &roll, &pitch, &yaww);
    printf("roll: %f \n",roll);
    printf("pitch: %f \n",pitch);
    printf("yaw: %f \n",yaww);

    *x=position.x;
    *y=position.y;
    *yaw=yaww;
}

void leggiValori(map<string,double> * mappavalori)
{
    ifstream filevalori;
    filevalori.open("/home/riccardo/ros_ws/src/g04-carraro-sassi-sturaro/param/points.pts");
    if(filevalori.is_open())
    {
        while (!filevalori.eof())
        {
            string linea;
            getline(filevalori,linea);
            string nome="";
            string valore="";
            int i=0;
            for(;i<linea.size();i++)
            {
                char c=linea.at(i);
                if(c!=' ')
                {
                    nome+=c;
                }
                else
                {
                    break;
                }
            }
            if(nome=="")
                continue;
            for(i=i+1;i<linea.size();i++)
            {
                char c=linea.at(i);
                if(c==' ')
                {
                    continue;
                }
                valore+=c;
            }
            printf("nome: %s, valore: %f \n",nome.c_str(),atof(valore.c_str()));
            mappavalori->insert(pair <string, double> (nome.c_str(), atof(valore.c_str())));
        }
    }
    filevalori.close();
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

void changeGlobalPlanner(char * name)
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::StrParameter enable_param;
  dynamic_reconfigure::Config conf;

  enable_param.name = "base_global_planner";
  enable_param.value = name;
  conf.strs.push_back(enable_param);
srv_req.config = conf;
ros::service::call("/move_base/set_parameters", srv_req, srv_resp);
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

void changeInflationLayerRadius(double local,double global)
{
  dynamic_reconfigure::ReconfigureRequest srv_req1;
  dynamic_reconfigure::ReconfigureResponse srv_resp1;
  dynamic_reconfigure::DoubleParameter enable_param1;
  dynamic_reconfigure::Config conf1;

  enable_param1.name = "inflation_radius";
  enable_param1.value = local;
  conf1.doubles.push_back(enable_param1);
srv_req1.config = conf1;
ros::service::call("/move_base/local_costmap/inflation_layer/set_parameters", srv_req1, srv_resp1);

dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter enable_param;
  dynamic_reconfigure::Config conf;

  enable_param.name = "inflation_radius";
  enable_param.value = global;
  conf.doubles.push_back(enable_param);
srv_req.config = conf;
ros::service::call("/move_base/global_costmap/inflation_layer/set_parameters", srv_req, srv_resp);
}

void changeInflationLayerCost(double local,double global)
{
  dynamic_reconfigure::ReconfigureRequest srv_req1;
  dynamic_reconfigure::ReconfigureResponse srv_resp1;
  dynamic_reconfigure::DoubleParameter enable_param1;
  dynamic_reconfigure::Config conf1;

  enable_param1.name = "cost_scaling_factor";
  enable_param1.value = local;
  conf1.doubles.push_back(enable_param1);
srv_req1.config = conf1;
ros::service::call("/move_base/local_costmap/inflation_layer/set_parameters", srv_req1, srv_resp1);

dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter enable_param;
  dynamic_reconfigure::Config conf;

  enable_param.name = "cost_scaling_factor";
  enable_param.value = global;
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

  dynamic_reconfigure::ReconfigureRequest srv_req1;
  dynamic_reconfigure::ReconfigureResponse srv_resp1;
  dynamic_reconfigure::DoubleParameter enable_param1;
  dynamic_reconfigure::Config conf1;

  enable_param1.name = "resolution";
  enable_param1.value = tol;
  conf1.doubles.push_back(enable_param1);
  srv_req1.config = conf1;
  ros::service::call("/move_base/global_costmap/set_parameters", srv_req1, srv_resp1);
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

void changeRotAcc(double acc)
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter enable_param;
  dynamic_reconfigure::Config conf;

  enable_param.name = "acc_lim_theta";
  enable_param.value = acc;
  conf.doubles.push_back(enable_param);
  srv_req.config = conf;
  ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);
}

void changeTransAcc(double acc)
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter enable_param;
  dynamic_reconfigure::Config conf;

  enable_param.name = "acc_lim_x";
  enable_param.value = acc;
  conf.doubles.push_back(enable_param);
  srv_req.config = conf;
ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req, srv_resp);

dynamic_reconfigure::ReconfigureRequest srv_req1;
  dynamic_reconfigure::ReconfigureResponse srv_resp1;
  dynamic_reconfigure::DoubleParameter enable_param1;
  dynamic_reconfigure::Config conf1;

  enable_param1.name = "acc_limit_trans";
  enable_param1.value = acc;
  conf1.doubles.push_back(enable_param1);
srv_req1.config = conf1;
ros::service::call("/move_base/DWAPlannerROS/set_parameters", srv_req1, srv_resp1);
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

void changeRobotPadding(double res)
{
  dynamic_reconfigure::ReconfigureRequest srv_req;
  dynamic_reconfigure::ReconfigureResponse srv_resp;
  dynamic_reconfigure::DoubleParameter enable_param;
  dynamic_reconfigure::Config conf;

  enable_param.name = "footprint_padding";
  enable_param.value = res;
  conf.doubles.push_back(enable_param);
srv_req.config = conf;
ros::service::call("/move_base/global_costmap/set_parameters", srv_req, srv_resp);


dynamic_reconfigure::ReconfigureRequest srv_req1;
  dynamic_reconfigure::ReconfigureResponse srv_resp1;
  dynamic_reconfigure::DoubleParameter enable_param1;
  dynamic_reconfigure::Config conf1;

  enable_param1.name = "footprint_padding";
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


void changeParametersFase1()
{
    //DWA
    thread t1(changeRotVel,0.1,2.0,0.1); //changeRotVel(0.1,2.0,0.1);
    thread t2(changeTransVel,0.1,1.0,0.1); //changeTransVel(0.1,1.0,0.1);
    thread t3(changeXVel,-1.0,1.0); //changeXVel(-1.0,1.0);
    thread t4(changeRotAcc,2.0); //changeRotAcc(2.0);
    thread t5(changeTransAcc,3.0); //changeTransAcc(3.0);

    thread t6(changeSimulationTime,4.0); //changeSimulationTime(4.0);
    thread t7(changeVTHSamples,50); //changeVTHSamples(40);
    thread t8(changeVXSamples,30); //changeVXSamples(20);

    thread t9(changeYawGoalTolerance,0.3); //changeYawGoalTolerance(0.3);
    thread t10(changeXYGoalTolerance,0.15); //changeXYGoalTolerance(0.15);

    thread t11(changePathDistanceBias,30); //changePathDistanceBias(30);
    thread t12(changeGoalDistanceBias,40); //changeGoalDistanceBias(40);
    thread t13(changeOccdistScale,5); //changeOccdistScale(5);
    thread t14(changeForwardPointDistance,0.325); //changeForwardPointDistance(0.325);

    thread t15(changePrunePlan,true); //changePrunePlan(true);

    //costmap

    thread t16(changeInflationLayerCost,1.75,1.75); //changeInflationLayerCost(1.75,1.75);
    thread t17(changeInflationLayerRadius,1.55,1.55); //changeInflationLayerRadius(1.55,1.55);

    thread t18(changeRobotPadding,0.03); //changeRobotPadding(0.03);

    //move_base

    thread t19(changeClearingRotation,false); //changeClearingRotation(false);
    thread t20(changeRecoveryBehavior,false); //changeRecoveryBehavior(false);

    changeGlobalPlanner("global_planner/GlobalPlanner");

    t1.join();
    t2.join();
    t3.join();
    t4.join();
    t5.join();
    t6.join();
    t7.join();
    t8.join();
    t9.join();
    t10.join();
    t11.join();
    t12.join();
    t13.join();
    t14.join();
    t15.join();
    t16.join();
    t17.join();
    t18.join();
    t19.join();
    t20.join();

}

void changeParametersFase2()
{
    //DWA

    thread t1(changeSimulationTime,2.0); //changeSimulationTime(2.0);

    thread t2(changeOccdistScale,0.02); //changeOccdistScale(0.02);

    //costmap

    thread t3(changeInflationLayerCost,10.0,10.0); //changeInflationLayerCost(10.0,10.0);
    thread t4(changeInflationLayerRadius,1.0,1.0); //changeInflationLayerRadius(1.0,1.0);

    thread t5(changeRobotPadding,0.005); //changeRobotPadding(0.01);

    t1.join();
    t2.join();
    t3.join();
    t4.join();
    t5.join();
    //move_base
}

void changeParametersFase3()
{
    //DWA

    thread t1(changeSimulationTime,4.0); //changeSimulationTime(4.0);
    thread t2(changeVTHSamples,70); //changeVTHSamples(70);
    thread t3(changeVXSamples,50); //changeVXSamples(50);

    thread t4(changeYawGoalTolerance,0.2); //changeYawGoalTolerance(0.2);
    thread t5(changeXYGoalTolerance,0.05); //changeXYGoalTolerance(0.05);

    thread t6(changeForwardPointDistance,0.2); //changeForwardPointDistance(0.2);

    //costmap

    //move_base

    t1.join();
    t2.join();
    t3.join();
    t4.join();
    t5.join();
    t6.join();
}

void changeParametersFase4()
{
    //DWA
    thread t1(changeRotAcc,1.0); //changeRotAcc(1.0);
    thread t2(changeTransAcc,1.0); //changeTransAcc(1.0);

    thread t3(changeYawGoalTolerance,0.15); //changeYawGoalTolerance(0.15);
    thread t4(changeXYGoalTolerance,0.03); //changeXYGoalTolerance(0.03);


    //costmap


    //move_base

    t1.join();
    t2.join();
    t3.join();
    t4.join();
}

void changeParametersFase5()
{
    //DWA
    thread t1(changeRotAcc,2.0); //changeRotAcc(2.0);
    thread t2(changeTransAcc,3.0); //changeTransAcc(3.0);

    thread t3(changeSimulationTime,2.0); //changeSimulationTime(2.0);

    thread t4(changeYawGoalTolerance,0.2); //changeYawGoalTolerance(0.2);
    thread t5(changeXYGoalTolerance,0.1); //changeXYGoalTolerance(0.1);
    thread t6(changeForwardPointDistance,0.325); //changeForwardPointDistance(0.325);

    //costmap


    //move_base

    t1.join();
    t2.join();
    t3.join();
    t4.join();
    t5.join();
    t6.join();
}


void changeParametersFase6()
{

}

void changeParametersFase7()
{
    //DWA

    thread t1(changeSimulationTime,4.0); //changeSimulationTime(4.0);
    thread t2(changeVTHSamples,40); //changeVTHSamples(40);
    thread t3(changeVXSamples,20); //changeVXSamples(20);


    thread t4(changeOccdistScale,5); //changeOccdistScale(5);


    //costmap

    thread t5(changeInflationLayerCost,1.75,1.75); //changeInflationLayerCost(1.75,1.75);
    thread t6(changeInflationLayerRadius,1.55,1.55); //changeInflationLayerRadius(1.55,1.55);

    thread t7(changeRobotPadding,0.03); //changeRobotPadding(0.03);

    //move_base

    t1.join();
    t2.join();
    t3.join();
    t4.join();
    t5.join();
    t6.join();
    t7.join();
}

void changeParametersFase8()
{
    //DWA
    thread t1(changeRotAcc,1.0); //changeRotAcc(1.0);
    thread t2(changeTransAcc,1.0); //changeTransAcc(1.0);

    thread t3(changeYawGoalTolerance,0.15); //changeYawGoalTolerance(0.15);
    thread t4(changeXYGoalTolerance,0.03); //changeXYGoalTolerance(0.03);
    thread t5(changeForwardPointDistance,0.2); //changeForwardPointDistance(0.2);


    thread t6(changeSimulationTime,3.0); //changeSimulationTime(3.0);
    thread t7(changeVTHSamples,70); //changeVTHSamples(70);
    thread t8(changeVXSamples,50); //changeVXSamples(50);

    thread t9(changeOccdistScale,0.02); //changeOccdistScale(0.02);

    //costmap
    thread t10(changeInflationLayerCost,10.0,10.0); //changeInflationLayerCost(10.0,10.0);
    thread t11(changeInflationLayerRadius,1.0,1.0); //changeInflationLayerRadius(1.0,1.0);
    thread t12(changeRobotPadding,0.01); //changeRobotPadding(0.01);

    //move_base

    t1.join();
    t2.join();
    t3.join();
    t4.join();
    t5.join();
    t6.join();
    t7.join();
    t8.join();
    t9.join();
    t10.join();
    t11.join();
    t12.join();
}


































void changeParametersOstacoliOLD()
{
    //DWA
    changeRotVel(0.1,2.0,0.1);
    changeTransVel(0.1,1.0,0.1);
    changeXVel(-1.0,1.0);
    changeRotAcc(2.0);
    changeTransAcc(3.0);

    changeSimulationTime(4.0);
    changeVTHSamples(40);
    changeVXSamples(20);

    changeYawGoalTolerance(0.3);
    changeXYGoalTolerance(0.15);

    changePathDistanceBias(10);
    changeGoalDistanceBias(50);
    changeOccdistScale(5);
    changeForwardPointDistance(0.1);

    changePrunePlan(true);

    //costmap

    changeInflationLayerCost(1.75,1.75);
    changeInflationLayerRadius(1.55,1.55);

    changeRobotPadding(0.03);

    //move_base

    changeClearingRotation(false);
    changeRecoveryBehavior(false);
}

void changeParametersPassaggioStrettoOLD()
{
    //DWA
    changeRotVel(0.1,2.0,0.1);
    changeTransVel(0.1,1.0,0.1);
    changeXVel(-1.0,1.0);
    changeRotAcc(2.0);
    changeTransAcc(1.0);

    changeSimulationTime(1.0);
    changeVTHSamples(20);
    changeVXSamples(10);

    changeYawGoalTolerance(0.5);
    changeXYGoalTolerance(0.2);

    changePathDistanceBias(30);
    changeGoalDistanceBias(40);
    changeOccdistScale(0.02);
    changeForwardPointDistance(0.3);

    changePrunePlan(true);

    //costmap

    changeInflationLayerCost(10.0,10.0);
    changeInflationLayerRadius(0.6,0.6);

    changeRobotPadding(0.01);

    //move_base

    changeClearingRotation(false);
    changeRecoveryBehavior(false);
}

void changeParametersDockingOLD()
{
    //DWA
    changeRotVel(0.1,2.0,0.1);
    changeTransVel(0.1,1.0,0.1);
    changeXVel(-1.0,1.0);
    changeRotAcc(2.0);
    changeTransAcc(1.0);

    changeSimulationTime(3.0);
    changeVTHSamples(70);
    changeVXSamples(50);

    changeYawGoalTolerance(0.15);
    changeXYGoalTolerance(0.03);

    changePathDistanceBias(20);
    changeGoalDistanceBias(40);
    changeOccdistScale(0.02);
    changeForwardPointDistance(0.1);

    changePrunePlan(true);

    //costmap

    changeInflationLayerCost(10.0,10.0);
    changeInflationLayerRadius(0.6,0.6);

    changeRobotPadding(0.001);

    //move_base

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

    for(int i=indexMinX;i<=indexMaxX;i++)
    {
        for(int j=indexMinY;j<=indexMaxY;j++)
        {
            if(mappa[i+j*width]!=0)
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

void plan(double startX, double startY, double startYaw, double xx, double yy, double yaww,double time)
{

    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE2StateSpace>());

    // set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    /*bounds.setLow(0,-offsetX);
    bounds.setHigh(0,offsetX);
    bounds.setLow(1,-offsetY);
    bounds.setHigh(1,offsetY);*/
    bounds.setLow(0,MIN(startX,xx)-(0.22*2));
    bounds.setHigh(0,MAX(startX,xx)+(0.22*2));
    bounds.setLow(1,MIN(startY,yy)-(0.22*2));
    bounds.setHigh(1,MAX(startY,yy)+(0.22*2));

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
    /*if(startYaw<0)
        startYaw=startYaw+2*3.14;*/
    start->setYaw(0.1);

    // create a goal state
    ob::ScopedState<ob::SE2StateSpace> goal(start);
    /*(*goal)[0]->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0.71;
    (*goal)[0]->as<ob::RealVectorStateSpace::StateType>()->values[1] = 2.0;
    (*goal)[1]->as<ob::SO2StateSpace::StateType>()->value = 0.0;*/
    (*goal)[0]->as<ob::RealVectorStateSpace::StateType>()->values[0] = xx;
    (*goal)[0]->as<ob::RealVectorStateSpace::StateType>()->values[1] = yy;
    (*goal)[1]->as<ob::SO2StateSpace::StateType>()->value = yaww;



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
    ob::PlannerStatus solved = planner->ob::Planner::solve(time);

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
    path.interpolate(8);
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
        mappa[((int)floor((x+offsetX)/resolution))+((int)floor((y+offsetY)/resolution))*width]=3;


    }

    if(stampaMappa)
    {
        for(int j=0;j<height;j++)
        {
            for(int i=0;i<width;i++)
            {
                char v=mappa[i+j*width];

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
}

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

  int c = getchar();  // read character (non-blocking)

  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
  return c;
}

int main(int argc, char ** argv)
{
    //avvio nodo
    ros::init(argc, argv, "programma");
    mappa=getCostMap(&width,&height,&resolution,&origin);
    offsetX=-origin.position.x;
    offsetY=-origin.position.y;
    pathLength=-1;

    map<string,double> valori;
    leggiValori(&valori);

    do{
        changeParametersFase1();
        printf("premere invio per continuare");
        getch();
        printf("da punto di partenza a passaggio stretto \n");
        //printf("VALORI: X:%f Y:%f YAW:%f",valori["narrowPassageStartX"],valori["narrowPassageStartY"],valori["narrowPassageStartYaw \n"]);
        double x1,y1,yaw1;
        getRobotPose(&x1,&y1,&yaw1);
        if(!sendGoalPose(valori["narrowPassageStartX"],valori["narrowPassageStartY"],valori["narrowPassageStartYaw"]))
        {
           sendGoalPose(valori["middlePointX"],valori["middlePointY"],valori["middlePointYaw"]);
            while(!sendGoalPose(valori["narrowPassageStartX"],valori["narrowPassageStartY"],valori["narrowPassageStartYaw"]));
        }

        printf("attraverso il passaggio stretto \n");
        double x2,y2,yaw2;
        getRobotPose(&x2,&y2,&yaw2);
        changeParametersFase2();
        double time=OMPL_time;
        if(pathLength==-1)
        {
            do{
                time++;
                plan(x2,y2,yaw2,valori["narrowPassageEndX"],valori["narrowPassageEndY"],valori["narrowPassageEndYaw"],time);
            }while (pathLength==-1);
        }
        for(int i=1;i<pathLength;i++)
        {
            int num=0;
            //if(i>=3 && i<=pathLength-3)
            //    continue;
            while(!sendGoalPose(pathX[i],pathY[i],valori["narrowPassageEndYaw"]) && num<numeroTentativi)
                num++;
        }
        //while(!sendGoalPose(narrowPassageEndX,narrowPassageEndY,narrowPassageEndYaw));

        printf("mi avvicino alla docking station \n");
        double x3,y3,yaw3;
        getRobotPose(&x3,&y3,&yaw3);
        changeParametersFase3();
        int tent=0;
        while(!sendGoalPose(valori["nearDocking2X"],valori["nearDocking2Y"],getYawFromQuaternion(0.0,0.0,valori["nearDocking2QZ"],valori["nearDocking2QW"])) && tent<numeroTentativi)
            tent++;

        printf("vado nella docking \n");
        double x4,y4,yaw4;
        getRobotPose(&x4,&y4,&yaw4);
        changeParametersFase4();
        int attempts=0;
        while(!sendGoalPose(valori["docking2X"],valori["docking2Y"],getYawFromQuaternion(0.0,0.0,valori["docking2QZ"],valori["docking2QW"])) && attempts<numeroTentativi)
            attempts++;

        changeParametersFase5();
        printf("premere invio per continuare");
        getch();

        printf("vado vicino al passaggio stretto \n");
        double x5,y5,yaw5;
        getRobotPose(&x5,&y5,&yaw5);
        double angle=valori["narrowPassageEndYaw"];
        if(angle>0)
            angle=angle-3.14;
        else
            angle=angle+3.14;
        while(!sendGoalPose(valori["narrowPassageEndX"],valori["narrowPassageEndY"],angle));

        printf("attraverso il passaggio stretto \n");
        double x6,y6,yaw6;
        getRobotPose(&x6,&y6,&yaw6);
        changeParametersFase6();
        attempts=0;
        time=OMPL_time;

        angle=valori["narrowPassageStartYaw"];
        if(angle>0)
            angle=angle-3.14;
        else
            angle=angle+3.14;

        /*do{
            time++;
            plan(x6,y6,yaw6,narrowPassageStartX,narrowPassageStartY,angle,time);
            attempts++;
        }while (pathLength==-1 && attempts<numeroTentativi+5);*/

        if(pathLength==-1)
        {
            double anglee=valori["narrowPassageStartYaw"];
            if(anglee>0)
                anglee=anglee-3.14;
            else
                anglee=anglee+3.14;
            while(!sendGoalPose(valori["narrowPassageStartX"],valori["narrowPassageStartY"],anglee));
        }
        else
        {
            for(int i=pathLength-1;i>=0;i--)
            {
                //if(i>=4 && i<=pathLength-4)
                //    continue;
                int num=0;
                while(!sendGoalPose(pathX[i],pathY[i],angle) && num<numeroTentativi)
                    num++;
            }
        }
        //while(!sendGoalPose(narrowPassageStartX,narrowPassageStartY,narrowPassageStartYaw));

        printf("vado vicino alla docking station \n");
        double x7,y7,yaw7;
        getRobotPose(&x7,&y7,&yaw7);
        changeParametersFase7();
        if(!sendGoalPose(valori["nearDocking1X"],valori["nearDocking1Y"],getYawFromQuaternion(0.0,0.0,valori["nearDocking1QZ"],valori["nearDocking1QW"])))
        {
            double angle=valori["middlePointYaw"];
            if(angle>0)
                angle=angle-3.14;
            else
                angle=angle+3.14;
            sendGoalPose(valori["middlePointX"],valori["middlePointY"],angle);
            tent=0;
            while(!sendGoalPose(valori["nearDocking1X"],valori["nearDocking1Y"],getYawFromQuaternion(0.0,0.0,valori["nearDocking1QZ"],valori["nearDocking1QW"])) && tent<numeroTentativi+5)
                tent++;
        }


        printf("vado nella docking \n");
        double x8,y8,yaw8;
        getRobotPose(&x8,&y8,&yaw8);
        changeParametersFase8();
        attempts=0;

        while(!sendGoalPose(valori["docking1X"],valori["docking1Y"],getYawFromQuaternion(0.0,0.0,valori["docking1QZ"],valori["docking1QW"])) && attempts<numeroTentativi)
            attempts++;

    }while(true);
    return 0;
}

