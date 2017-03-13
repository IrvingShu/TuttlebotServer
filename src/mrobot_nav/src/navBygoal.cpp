#include "ros/ros.h"
#include "std_msgs/String.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <pthread.h>
#include <fcntl.h>
#include <sys/wait.h>
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

MoveBaseClient *ac;
char *host_name = "127.0.0.1";
int port = 8000;
int socket_descriptor;  


int SocketSend(int clientSocketFd ,char *buf, int len)
{
    int ret = 0;
    if(clientSocketFd <= 0){
        return -1;
    }
    ret = send(clientSocketFd, buf, len, 0);
    if(ret <=0)
    {
        return -1;
    }else if(ret == len)
    {
        return 0;
    }
    return 0;
}

void arriveGoalCallback(const move_base_msgs::MoveBaseGoal &goal)
{
    SocketSend(socket_descriptor, "r", 1);
    ROS_INFO("Sending goal");
    ac->sendGoal(goal);
    ac->waitForResult();
    if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        char *tmp = "s";
        ROS_INFO("arrive goal!");
        SocketSend(socket_descriptor, tmp, 1);
        //发送状态到达了终点
    }
    else
    {
        SocketSend(socket_descriptor, "s", 1);
        ROS_INFO("arrive fail!"); 
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_navigation_goals");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("server", 1000, arriveGoalCallback);
    //socket init
    struct sockaddr_in pin;  
    bzero(&pin,sizeof(pin));  
    pin.sin_family = AF_INET;  
    inet_pton(AF_INET,host_name,&pin.sin_addr);  
    pin.sin_port = htons(port);  
    if((socket_descriptor =  socket(AF_INET,SOCK_STREAM,0)) == -1)  
    {  
        perror("error opening socket \n");  
        exit(1);  
    }  
    if(connect(socket_descriptor,(struct sockaddr * )&pin,sizeof(pin)) == -1)  
    {  
        perror("error connecting to socket \n");  
        exit(1);  
    }
    

    //start navigation
    ac = new MoveBaseClient("move_base", true);
    while(!ac->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    ros::spin();
    ROS_INFO("spin end!");
    return 0;
}