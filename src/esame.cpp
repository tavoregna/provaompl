#define numeroTentativi 3
#define PUBLISHER_TOPIC "/turtlebot/mobile_base/commands/velocity"
#define SUBSCRIBER_TOPIC "/turtlebot/scan"
#define WALL_DISTANCE 0.18
#define MAX_SPEED 0.3
#define LOCALIZATION_MESSAGE "amcl_pose"                   //"poseupdate"

#include <iostream>
#include <math.h>
#include "ros/ros.h"
#include <stdio.h>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <map>
#include <thread>
#include <termios.h>

#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Point.h"

#include <move_base_msgs/MoveBaseAction.h>

#include <actionlib/client/simple_action_client.h>

#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/StrParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

using namespace std;

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

    double roll,pitch,yaw;
    toEulerAngle(q,&roll,&pitch,&yaw);
    return yaw;
}

//nel sistema di riferimento /map
void getRobotPose(double* x, double* y, double* yaw,bool print)
{
    geometry_msgs::PoseWithCovarianceStamped msg;

    msg=*(ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>(LOCALIZATION_MESSAGE));

    geometry_msgs::Point position=msg.pose.pose.position;
    geometry_msgs::Quaternion orientation=msg.pose.pose.orientation;

    double roll,pitch,yaww;
    toEulerAngle(orientation, &roll, &pitch, &yaww);

    *x=position.x;
    *y=position.y;
    *yaw=yaww;

    if(print)
    {
        printf("position: \n\t");
        printf("x: %f \n\t",position.x);
        printf("y: %f \n\t",position.y);
        printf("z: %f \n",position.z);
        printf("orientation: \n\t");
        printf("QUATERNION: \n\t\t");
        printf("x: %f \n\t\t",orientation.x);
        printf("y: %f \n\t\t",orientation.y);
        printf("z: %f \n\t\t",orientation.z);
        printf("w: %f \n\t",orientation.w);
        printf("ROLL,PITCH,YAW: \n\t\t");
        printf("roll: %f \n\t\t",roll);
        printf("pitch: %f \n\t\t",pitch);
        printf("yaw: %f \n",yaww);
    }
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

    goal.target_pose.pose.position.x = x;
    goal.target_pose.pose.position.y = y;

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

    thread t11(changePathDistanceBias,10); //changePathDistanceBias(30);
    thread t12(changeGoalDistanceBias,50); //changeGoalDistanceBias(40);
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

    //move_base

    t1.join();
    t2.join();
    t3.join();
    t4.join();
    t5.join();
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

    //leggo da file le posizioni dei punti nella mappa e li metto nella Hashmap "valori"
    map<string,double> valori;
    leggiValori(&valori);

    do{
        //FASE 1: da punto di partenza a passaggio stretto

        changeParametersFase1();
        printf("premere invio per continuare");
        getch();
        printf("da punto di partenza a passaggio stretto \n");
        if(!sendGoalPose(valori["narrowPassageStartX"],valori["narrowPassageStartY"],valori["narrowPassageStartYaw"]))
        {
           sendGoalPose(valori["middlePointX"],valori["middlePointY"],valori["middlePointYaw"]);
            while(!sendGoalPose(valori["narrowPassageStartX"],valori["narrowPassageStartY"],valori["narrowPassageStartYaw"]));
        }


        //FASE 2: attraversamento passaggio stretto

        printf("attraverso il passaggio stretto \n");
        changeParametersFase2();
        double xF2,yF2,yawF2;
        system("roslaunch dem_wall_following wall_following_dx.launch &");
        do{
            ros::Duration(0.3).sleep();
            getRobotPose(&xF2,&yF2,&yawF2,false);
        }while(!(xF2>=valori["PSx"] && yF2>=valori["PS1middleY"]));
        system("rosnode kill wallFollowing");

        ros::Duration(0.3).sleep();

        system("roslaunch dem_wall_following wall_following_sx.launch &");
        do{
            ros::Duration(0.3).sleep();
            getRobotPose(&xF2,&yF2,&yawF2,false);
        }while(!(xF2>=valori["PSx"] && yF2>=valori["PS1endY"]));
        system("rosnode kill wallFollowing");
        ros::Duration(0.3).sleep();


        //FASE 3: avvicinamento alla docking station

        printf("mi avvicino alla docking station \n");
        changeParametersFase3();
        int tentativiF3=0;
        while(!sendGoalPose(valori["nearDocking2X"],valori["nearDocking2Y"],getYawFromQuaternion(0.0,0.0,valori["nearDocking2QZ"],valori["nearDocking2QW"])) && tentativiF3<numeroTentativi)
            tentativiF3++;


        //FASE 4: posizionamento nella docking station

        printf("vado nella docking \n");
        changeParametersFase4();
        int tentativiF4=0;
        while(!sendGoalPose(valori["docking2X"],valori["docking2Y"],getYawFromQuaternion(0.0,0.0,valori["docking2QZ"],valori["docking2QW"])) && tentativiF4<numeroTentativi)
            tentativiF4++;


        //FASE 5: ritorno nel passaggio stretto

        changeParametersFase5();
        printf("premere invio per continuare");
        getch();
        printf("vado vicino al passaggio stretto \n");
        double angle=valori["narrowPassageEndYaw"];
        if(angle>0)
            angle=angle-3.14;
        else
            angle=angle+3.14;
        while(!sendGoalPose(valori["narrowPassageEndX"],valori["narrowPassageEndY"],angle));


        //FASE 6: attraversamento passaggio stretto

        printf("attraverso il passaggio stretto \n");
        changeParametersFase6();
        double xF6,yF6,yawF6;
        system("roslaunch dem_wall_following wall_following_dx.launch &");
        do{
            ros::Duration(0.3).sleep();
            getRobotPose(&xF6,&yF6,&yawF6,false);
        }while(!(xF6>=valori["PSx"] && yF6<=valori["PS2middleY"]));
        system("rosnode kill wallFollowing");

        ros::Duration(0.3).sleep();

        system("roslaunch dem_wall_following wall_following_sx.launch &");
        do{
            ros::Duration(0.3).sleep();
            getRobotPose(&xF6,&yF6,&yawF6,false);
        }while(!(xF6>=valori["PSx"] && yF6<=valori["PS2endY"]));
        system("rosnode kill wallFollowing");
        ros::Duration(0.3).sleep();


        //FASE 7: avvicinamento alla docking station

        printf("vado vicino alla docking station \n");
        changeParametersFase7();
        if(!sendGoalPose(valori["nearDocking1X"],valori["nearDocking1Y"],getYawFromQuaternion(0.0,0.0,valori["nearDocking1QZ"],valori["nearDocking1QW"])))
        {
            double angle=valori["middlePointYaw"];
            if(angle>0)
                angle=angle-3.14;
            else
                angle=angle+3.14;
            sendGoalPose(valori["middlePointX"],valori["middlePointY"],angle);
            int tentativiF7=0;
            while(!sendGoalPose(valori["nearDocking1X"],valori["nearDocking1Y"],getYawFromQuaternion(0.0,0.0,valori["nearDocking1QZ"],valori["nearDocking1QW"])) && tentativiF7<numeroTentativi+5)
                tentativiF7++;
        }


        //FASE 8: posizionamento nella docking station

        printf("vado nella docking \n");
        changeParametersFase8();
        int tentativiF8=0;

        while(!sendGoalPose(valori["docking1X"],valori["docking1Y"],getYawFromQuaternion(0.0,0.0,valori["docking1QZ"],valori["docking1QW"])) && tentativiF8<numeroTentativi)
            tentativiF8++;

    }while(true);
    return 0;
}

