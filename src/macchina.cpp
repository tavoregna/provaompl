#define raggio 0.18

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

/*void moveToPoint(ros::NodeHandle n, geometry_msgs::Point goal_point, double distance_tolerance,double angle_tolerance)
{
    ros::Publisher cmd_pub = n.advertise<geometry_msgs::Twist>("turtlebot/mobile_base/commands/velocity", 1);
    geometry_msgs::Twist twist;
    printf("position goal: \n");
    printf("x: %f \n",goal_point.x);
    printf("y: %f \n",goal_point.y);
    geometry_msgs::Pose turtlebot_pose=getRobotPose_1();
    double turtlebot_theta_goal=atan2(((goal_point.y)-(turtlebot_pose.position.y)),((goal_point.x)-(turtlebot_pose.position.x)));
    double turtlebot_theta=euler_from_quaternion(turtlebot_pose.orientation);
    printf("+++++ theta_goal: %f \n",turtlebot_theta_goal);
    ros::Rate loop_rate(10);
    while(getDistance(turtlebot_pose.position,goal_point)>distance_tolerance)
    {
        if(abs(turtlebot_theta_goal-turtlebot_theta)>angle_tolerance)
        {
            twist.linear.x=0;
            twist.angular.z=0;
            cmd_pub.publish(twist);
            while(abs(turtlebot_theta_goal-turtlebot_theta)>angle_tolerance)
            {
                twist.linear.x=0;
                twist.angular.z=0.1;
                cmd_pub.publish(twist);
                loop_rate.sleep();
                turtlebot_pose=getRobotPose_1();
                turtlebot_theta=euler_from_quaternion(turtlebot_pose.orientation);
            }
            twist.linear.x=0;
            twist.angular.z=0;
        }
        else
        {
            twist.linear.x=0.1;
            twist.linear.y=0;
            twist.linear.z=0;
            twist.angular.x=0;
            twist.angular.y=0;
            twist.angular.z=0;
        }
        cmd_pub.publish(twist);
        loop_rate.sleep();
        turtlebot_pose=getRobotPose_1();
        turtlebot_theta=euler_from_quaternion(turtlebot_pose.orientation);
    }
    twist.linear.x=0;
    twist.angular.z=0;
    cmd_pub.publish(twist);
}*/

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
    bounds.setLow(0,-offsetX);
    bounds.setHigh(0,offsetX);
    bounds.setLow(1,-offsetY);
    bounds.setHigh(1,offsetY);

    space->setBounds(bounds);

    space->setLongestValidSegmentFraction(0.0001);

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
    
    si->setStateValidityCheckingResolution(0.0001);

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
    (*goal)[0]->as<ob::RealVectorStateSpace::StateType>()->values[0] = 0.71;
    (*goal)[0]->as<ob::RealVectorStateSpace::StateType>()->values[1] = 2.0;
    (*goal)[1]->as<ob::SO2StateSpace::StateType>()->value = 0.0;




    // create a problem instance
    auto pdef(std::make_shared<ob::ProblemDefinition>(si));

    // set the start and goal states
    pdef->setStartAndGoalStates(start, goal, 0.0001);

    pdef->setOptimizationObjective(getPathLengthObjective(si));

    // create a planner for the defined space

    //auto planner(std::make_shared<ompl::geometric::pSBL>(si));
    auto planner(std::make_shared<ompl::geometric::RRTConnect>(si));
    //auto planner(std::make_shared<ompl::geometric::RRTstar>(si));
    //auto planner(std::make_shared<ompl::geometric::BKPIECE1>(si));
    //auto planner(std::make_shared<oc::EST>(si));
    //auto planner(std::make_shared<oc::KPIECE1>(si));
    //auto planner(std::make_shared<ompl::geometric::SPARStwo>(si));
    //auto planner(std::make_shared<ompl::geometric::RRTsharp>(si));


    // set the problem we are trying to solve for the planner
    planner->setProblemDefinition(pdef);

    // perform setup steps for the planner
    planner->setup();


    // print the settings for this space
    si->printSettings(std::cout);

    // print the problem settings
    pdef->print(std::cout);

    // attempt to solve the problem within ten seconds of planning time
    ob::PlannerStatus solved = planner->ob::Planner::solve(20.0);

    ob::PathPtr pathh;

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

    for(int i=0;i<path.getStateCount();i++)
    {
        const auto *se2state = (path.getState(i))->as<ob::SE2StateSpace::StateType>();

        const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);
        const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1);

        double x=(*pos).values[0];
        double y=(*pos).values[1];

        printf("%d: X: %d, Y: %d\n",i+1,(int)floor((x+offsetX)/resolution),(int)floor((y+offsetY)/resolution));
        map[((int)floor((x+offsetX)/resolution))+((int)floor((y+offsetY)/resolution))*width]=3;


    }

    for(int j=0;j<height;j++)
    {
        for(int i=0;i<width;i++)
        {
            char v=map[i+j*width];

            /* if(i==((int)floor((startX+offsetX)/resolution)) && j==((int)floor((startY+offsetY)/resolution)))
            {
                printf("\x1b[36m%s\x1b[0m","@");
                continuee=false;
            }*/

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


/*void planWithSimpleSetup()
{
    // construct the state space we are planning in
    auto space(std::make_shared<ob::SE2StateSpace>());

    // set the bounds for the R^2 part of SE(2)
    ob::RealVectorBounds bounds(2);
    //bounds.setLow(-1);
    //bounds.setHigh(1);
    bounds.setLow(0,-3);
    bounds.setHigh(0,width);
    bounds.setLow(1,-3);
    bounds.setHigh(1,height);

    space->setBounds(bounds);

    // create a control space
    auto cspace(std::make_shared<oc::RealVectorControlSpace>(space, 2));
    // set the bounds for the control space
    ob::RealVectorBounds cbounds(2);
    //cbounds.setLow(-0.3);
    //cbounds.setHigh(0.3);
    cbounds.setLow(0,-0.3);
    cbounds.setHigh(0,0.3);
    cbounds.setLow(1,-0.3);
    cbounds.setHigh(1,0.3);

    cspace->setBounds(cbounds);

    // define a simple setup class
    oc::SimpleSetup ss(cspace);

    // set the state propagation routine
    ss.setStatePropagator(propagate);

    // set state validity checking for this space
    ss.setStateValidityChecker(
                [&ss](const ob::State *state) { return isStateValid(ss.getSpaceInformation().get(), state); });

    // create a start state
    ob::ScopedState<ob::SE2StateSpace> start(space);
    start->setX(startX);
    start->setY(startY);
    start->setYaw(startYaw);

    // create a  goal state; use the hard way to set the elements
    ob::ScopedState<ob::SE2StateSpace> goal(space);
    (*goal)[0]->as<ob::RealVectorStateSpace::StateType>()->values[0] = 1.0;
    (*goal)[0]->as<ob::RealVectorStateSpace::StateType>()->values[1] = 2.0;
    (*goal)[1]->as<ob::SO2StateSpace::StateType>()->value = 0.0;


    // set the start and goal states
    ss.setStartAndGoalStates(start, goal, 0.001);

    // ss.setPlanner(std::make_shared<oc::PDST>(ss.getSpaceInformation()));
    // ss.getSpaceInformation()->setMinMaxControlDuration(1,100);
    // attempt to solve the problem within one second of planning time
    ob::PlannerStatus solved = ss.solve(20.0);

    if (solved)
    {
        std::cout << "Found solution:" << std::endl;
        // print the path to screen

        ss.getSolutionPath().printAsMatrix(std::cout);
    }
    else{
        std::cout << "No solution found" << std::endl;
        return;
    }

     ompl::control::PathControl path=ss.getSolutionPath();


     for(int j=0;j<height;j++)
     {
         for(int i=0;i<width;i++)
         {
             char v=map[i+j*width];

             bool continuee=true;

             for(int i=0;i<path.getStateCount();i++)
             {
                 const auto *se2state = (path.getState(i))->as<ob::SE2StateSpace::StateType>();

                 const auto *pos = se2state->as<ob::RealVectorStateSpace::StateType>(0);
                 const auto *rot = se2state->as<ob::SO2StateSpace::StateType>(1);

                 double x=(*pos).values[0];
                 double y=(*pos).values[1];

                 if(i==((int)floor((x+offsetX)/resolution)) && j==((int)floor((y+offsetY)/resolution)))
                 {
                     printf("\x1b[36m%s\x1b[0m","@");
                     continuee=false;
                     break;
                 }

             }

             if(i==((int)floor((startX+offsetX)/resolution)) && j==((int)floor((startY+offsetY)/resolution)) && continuee)
             {
                 printf("\x1b[36m%s\x1b[0m","@");
                 continuee=false;
             }

             if(continuee)
             {
             if(v==-1)
                 printf("\x1b[30m%-d\x1b[0m",0);
             else if(v==0)
                 printf("\x1b[32m%-d\x1b[0m",1);
             else
                 printf("\x1b[33m%-d\x1b[0m",2);
             }

         }
         printf("\n");

     }
}*/



int main(int argc, char ** argv)
{

    /*std::cout << "OMPL version: " << OMPL_VERSION << std::endl;

    plan();

    std::cout << std::endl << std::endl;*/

    ros::init(argc, argv, "programma");

    map=getCostMap(&width,&height,&resolution,&origin);
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
    printf("w: %f\n",origin.orientation.w);

    offsetX=-origin.position.x;
    offsetY=-origin.position.y;
    getRobotPose();
    plan();

    printf("FLOOR 1: %d\n",(int)floor((startX+offsetX)/resolution));
    printf("FLOOR 2: %d\n",(int)floor((startY+offsetY)/resolution));

    /*for(int j=0;j<height;j++)
    {
        for(int i=0;i<width;i++)
        {
            char v=map[i+j*width];
            if(i==((int)floor((startX+offsetX)/resolution)) && j==((int)floor((startY+offsetY)/resolution)))
                printf("\x1b[36m%s\x1b[0m","@");
            else if(i==((int)floor((startX-raggio+offsetX)/resolution)) && j==((int)floor((startY-raggio+offsetY)/resolution)))
                printf("\x1b[36m%s\x1b[0m","@");
            else if(i==((int)floor((startX-raggio+offsetX)/resolution)) && j==((int)floor((startY+raggio+offsetY)/resolution)))
                printf("\x1b[36m%s\x1b[0m","@");
            else if(i==((int)floor((startX+raggio+offsetX)/resolution)) && j==((int)floor((startY-raggio+offsetY)/resolution)))
                printf("\x1b[36m%s\x1b[0m","@");
            else if(i==((int)floor((startX+raggio+offsetX)/resolution)) && j==((int)floor((startY+raggio+offsetY)/resolution)))
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

    /* for(int j=0;j<height;j++)
    {
        for(int i=0;i<width;i++)
        {
            char v=map[i+j*width];
            if(i==((int)floor((1+offsetX)/resolution)) && j==((int)floor((2+offsetY)/resolution)))
                printf("\x1b[36m%s\x1b[0m","@");
            else if(i==((int)floor((1-raggio+offsetX)/resolution)) && j==((int)floor((2-raggio+offsetY)/resolution)))
                printf("\x1b[36m%s\x1b[0m","@");
            else if(i==((int)floor((1-raggio+offsetX)/resolution)) && j==((int)floor((2+raggio+offsetY)/resolution)))
                printf("\x1b[36m%s\x1b[0m","@");
            else if(i==((int)floor((1+raggio+offsetX)/resolution)) && j==((int)floor((2-raggio+offsetY)/resolution)))
                printf("\x1b[36m%s\x1b[0m","@");
            else if(i==((int)floor((1+raggio+offsetX)/resolution)) && j==((int)floor((2+raggio+offsetY)/resolution)))
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

    return 0;
}
