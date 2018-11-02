#include "ros/ros.h"
#include "sound_play/SoundRequest.h"
#include "sound_play/SoundRequestGoal.h"
#include "sound_play/SoundRequestActionGoal.h"
#include <actionlib/client/simple_action_client.h>


void songManager(std::string path, int comm,double volume)
{
    ros::NodeHandle n;

    sound_play::SoundRequest sr;
    sound_play::SoundRequestActionGoal srag;

    sr.sound = -2;
    sr.command = comm;
    sr.volume = volume;
    sr.arg = path;
    sr.arg2 = "";

    srag.header.seq=0;
    srag.header.stamp.sec=0;
    srag.header.stamp.nsec=0;
    srag.header.frame_id="";

    srag.goal_id.stamp.sec=0;
    srag.goal_id.stamp.nsec=0;
    srag.goal_id.id="";

    srag.goal.sound_request =  sr;

    n.setParam("soundPlay/received",0);
    ros::Publisher soundPub = n.advertise<sound_play::SoundRequestActionGoal>("sound_play/goal", 10, true);
    int initialSub = soundPub.getNumSubscribers();
    while(soundPub.getNumSubscribers()==initialSub+1)
    {
        ros::Duration(0.2).sleep();
    }
    ros::Duration(0.2).sleep();

    soundPub.publish(srag);

    ros::spinOnce();

    int received=0;

    do{
       ros::Duration(0.2).sleep();
       ros::spinOnce();
       n.getParam("soundPlay/received",received);
    }while(received == 0);

    n.shutdown();
}

void playSong(std::string path, double volume)
{
    songManager(path, 1, volume);
}

void stopSong(std::string path, double volume)
{
    songManager(path, 0, volume);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "soundPlay");

    playSong("/home/riccardo/Scrivania/canzoni/sonomarrtino.wav",1.0);


ros::Duration(3).sleep();

stopSong("/home/riccardo/Scrivania/canzoni/sonomarrtino.wav",1.0);

ros::Duration(1).sleep();

    playSong("/home/riccardo/Scrivania/canzoni/imperial.wav",1.0);
    ros::Duration(2).sleep();
    playSong("/home/riccardo/Scrivania/canzoni/sonomarrtino.wav",1.0);

ros::Duration(10).sleep();
    
stopSong("/home/riccardo/Scrivania/canzoni/imperial.wav",1.0);
    
}
