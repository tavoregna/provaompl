#include "ros/ros.h"
#include "sound_play/SoundRequest.h"
#include "sound_play/SoundRequestGoal.h"
#include "sound_play/SoundRequestActionGoal.h"
#include <actionlib/client/simple_action_client.h>

void soundListenerCallback(const sound_play::SoundRequestActionGoal& msg)
{
    ROS_ERROR("ricevuto");
    ros::NodeHandle n;
    n.setParam("soundPlay/received",1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "soundListener");
    
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("sound_play/goal", 1000, soundListenerCallback);

    ros::spin();

    return 0;
}
