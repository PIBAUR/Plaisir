#include <ros/ros.h>
#include <unistd.h>
#include <stdio.h>
#include <signal.h>
#include <sys/wait.h>
#include "sstream"
#include <scenario_msgs/MediaArray.h>


int nb_media;
int index_media;
scenario_msgs::MediaArray medias_msg;
bool is_parent;


void mediaArrayCB(const scenario_msgs::MediaArray& msg);
void play_media(scenario_msgs::Media &media);
void child_play_media(scenario_msgs::Media &media);
void parent_wait_child(pid_t pid_child);

void mediaArrayCB(const scenario_msgs::MediaArray& msg)
{
    if(is_parent)
    {
        medias_msg = msg;
        nb_media = msg.medias.size();
        index_media = 0;
    }
}


void child_play_media(scenario_msgs::Media &media)
{

    is_parent = false;

    std::stringstream uri_path;
    uri_path << "uri=file://" << media.path;
    if(!ros::ok())
    {
        EXIT_SUCCESS;
    }
    ROS_INFO_STREAM("Starting Video");

    //system("gst-launch-1.0 playbin uri=file:///home/serveur/videosample/768p_12fps.mp4");
    execlp("gst-launch-1.0", "gst-launch-1.0", "playbin", uri_path.str().c_str(), NULL);
    EXIT_SUCCESS;
}


void parent_wait_child(pid_t pid_child)
{
    ROS_INFO_STREAM("Waiting video end...");
    int status;
    pid_t w;
    do
    {
        ros::spinOnce();
        w = waitpid(pid_child, &status, WUNTRACED | WCONTINUED);
        if (w == -1)
        {
            ROS_ERROR_STREAM("waitpid");
            exit(EXIT_FAILURE);
        }
        if (WIFEXITED(status))
        {
            ROS_INFO_STREAM("exited, status= " << WEXITSTATUS(status));
        }
        else if (WIFSIGNALED(status))
        {
            ROS_WARN_STREAM("killed by signal " << WTERMSIG(status));
        }
        else if (WIFSTOPPED(status))
        {
            ROS_WARN_STREAM("stopped by signal " << WSTOPSIG(status));
        }
        else if (WIFCONTINUED(status))
        {
            ROS_INFO_STREAM("continued");
        }
    }while (!WIFEXITED(status) && !WIFSIGNALED(status) && ros::ok());

    if(!ros::ok())
    {
        kill(pid_child,SIGTERM);
        ROS_WARN_STREAM("Starting Video");
        waitpid(pid_child, &status, WUNTRACED | WCONTINUED);
    }
}


void play_media(scenario_msgs::Media &media)
{
    pid_t pid_player = fork();
    switch(pid_player)
    {
        case -1: //erreur...
            ROS_ERROR_STREAM("error\n\n");
            break;
        case 0://Fils = player
            child_play_media(media);
            break;
        default: //Pere = wait for player exit
            parent_wait_child(pid_player);
            break;
    }
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "videoplayer");
    ros::NodeHandle nh;
    ROS_INFO_STREAM("GoodMorning");
    ros::Subscriber media_sub = nh.subscribe("media", 1, mediaArrayCB);

    nb_media = 0;
    index_media = 0;
    is_parent = true;

    while(ros::ok())
    {
        if(nb_media!=0 && index_media < nb_media)
        {
            play_media(medias_msg.medias[index_media]);
            index_media++;
        }
        ros::spinOnce();
    }
    ROS_INFO_STREAM("Goodbye!");
    return 0;
}

