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
#include <iostream>
#include <sstream>
#include <vector>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

std::vector<move_base_msgs::MoveBaseGoal> place;
ros::Publisher server_pub;

void initMap()
{
    move_base_msgs::MoveBaseGoal goal_1;
    goal_1.target_pose.header.frame_id = "map";
    goal_1.target_pose.header.stamp = ros::Time::now();
    goal_1.target_pose.pose.position.x = 7.0;
    goal_1.target_pose.pose.position.y = 5.0;
    goal_1.target_pose.pose.orientation.z = 0.670634;
    goal_1.target_pose.pose.orientation.w = 0.741783;

    move_base_msgs::MoveBaseGoal goal_2;
    goal_2.target_pose.header.frame_id = "map";
    goal_2.target_pose.header.stamp = ros::Time::now();
    goal_2.target_pose.pose.position.x = 2.04;
    goal_2.target_pose.pose.position.y = 3.78;
    goal_2.target_pose.pose.orientation.z = 0.82855;
    goal_2.target_pose.pose.orientation.w = 0.55990715;

    move_base_msgs::MoveBaseGoal goal_3;
    goal_3.target_pose.header.frame_id = "map";
    goal_3.target_pose.header.stamp = ros::Time::now();
    goal_3.target_pose.pose.position.x = 2.04673671;
    goal_3.target_pose.pose.position.y = 7.06953681;
    goal_3.target_pose.pose.orientation.z = 0.963018;
    goal_3.target_pose.pose.orientation.w = 0.269436;

    place.push_back(goal_1);
    place.push_back(goal_2);
    place.push_back(goal_3);

}

//发送函数
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
    /*
    while(ret < len)
    {
        ROS_INFO("%d",ret);
        ret += send(clientSocketFd, buf+ret, len-ret, 0);
    }*/
    return 0;
}

void* rec_data(void *fd)
{

    int client_sockfd;
    int i, byte;
    char char_recv[100];
    client_sockfd = *((int *) fd);
    while(1)
    {
        if((byte=recv(client_sockfd, char_recv, 100, 0)) == -1)
        {
            ROS_INFO("recv failed!");
            exit(-1);
        }
        if(strcmp(char_recv, "exit") == 0)
        {
            break;
        }

        move_base_msgs::MoveBaseGoal goal;

        //goal = place[(int)char_recv[0]];
        switch(char_recv[0])
        {
            case '0': goal = place[0];break;
            case '1': goal = place[1];break;
            case '2': goal = place[2]; break;
            case '3': goal = place[3]; break;
            case '4': goal = place[4]; break;
            case '5': goal = place[5]; break;
            case '6': goal = place[6]; break;
            case '7': goal = place[7]; break;
            case '8': goal = place[8]; break;
            case '9': goal = place[9]; break;
        }
        
        server_pub.publish(goal);

        ROS_INFO("receive from client is:  %s\n", char_recv);
        SocketSend(client_sockfd ,char_recv, strlen(char_recv));
        
        /*
        MoveBaseClient ac("move_base", true);
        while(!ac.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("Waiting for the move_base action server to come up");
        }
        ROS_INFO("Sending goal");
        ac.sendGoal(goal);
        ac.waitForResult();
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            ROS_INFO("arrive goal!");
        }
        else
        {
            ROS_INFO("arrive fail!"); 
        }*/
           

    }
    free(fd);
    close(client_sockfd);
    pthread_exit(NULL);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "serverTalker");
    ros::NodeHandle n;
    server_pub = n.advertise<move_base_msgs::MoveBaseGoal>("server", 1000);


    initMap();

    int server_sockfd;
    int *client_sockfd;
    int server_len, client_len;
    struct sockaddr_in server_address;
    struct sockaddr_in client_address;
    struct sockaddr_in tempaddress;
    int i, byte;
    char char_recv, char_send;
    socklen_t templen;
    server_sockfd = socket(AF_INET, SOCK_STREAM, 0);//create socket

    server_address.sin_family = AF_INET;
    server_address.sin_addr.s_addr = htonl(INADDR_ANY);
    server_address.sin_port = htons(8000);
    server_len = sizeof(server_address);

    if(bind(server_sockfd, (struct sockaddr*)& server_address, server_len) == -1)
    {
        ROS_INFO("bind failed!");
        return -1;
    }
    if(listen(server_sockfd, 5) == -1)
    {
        ROS_INFO("listen failed!");
        return -1;
    }

    templen = sizeof(struct sockaddr);
    ROS_INFO("server waiting for connect\n");

    while(1)
    {
        pthread_t thread;
        client_sockfd = (int*) malloc(sizeof(int));   
        client_len = sizeof(client_address);
        *client_sockfd = accept(server_sockfd, (struct sockaddr*)& client_address, (socklen_t *)& client_len);
        if(-1 == *client_sockfd)
        {
            ROS_INFO("accept failed!");
            continue;
        }
        if(pthread_create(&thread, NULL, rec_data, client_sockfd) != 0)
        {
            ROS_INFO("create thread failed!");
            break;
        }
    }
    shutdown(*client_sockfd, 2);
    shutdown(server_sockfd, 2);

    return 0;
}