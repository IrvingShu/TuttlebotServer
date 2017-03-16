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
//robot is busy

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
    ROS_INFO("ret: %d, len: %d", ret,len);
    if(ret <=0)
    {
        return -1;
    }else if(ret == len)
    {
        ROS_INFO("sned success!");
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

volatile int robot_is_busy = 0;

void* rec_data(void *fd)
{

    int client_sockfd;
    int i, byte;
    client_sockfd = *((int *) fd);
    while(1)
    {
        char char_recv[100];
        if((byte=recv(client_sockfd, char_recv, 100, 0)) == -1)
        {
            ROS_INFO("recv failed!");
            //exit(-1);
            //free(fd);
            //close(client_sockfd);
            //pthread_exit(NULL);
            break;
        }
        ROS_INFO("receive from client is:  %s\n", char_recv);

        if(char_recv[0] == 'r')
        {
            ROS_INFO("ros is running!");
            robot_is_busy = 1;
            
        }
        if(char_recv[0] == 's')
        {
            ROS_INFO("ros is stopped!");
            robot_is_busy = 0;
            char msg[] = "f";
            SocketSend(client_sockfd, msg, strlen(msg));
            
        }
        if(strcmp(char_recv, "exit") == 0)
        {
            break;
        }

        if(robot_is_busy == 1)
        {
            char msg[] = "r";
            ROS_INFO("robot busy!");
            SocketSend(client_sockfd, msg, strlen(msg));
            char_recv[0] = 'c';
        }else if(robot_is_busy==0){
            ROS_INFO("robot start!");
            move_base_msgs::MoveBaseGoal goal;
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
                default: continue;
            }
            //send(client_sockfd, char_recv, strlen(char_recv),0);
            SocketSend(client_sockfd ,char_recv,strlen(char_recv)); 
            ROS_INFO("send client!");
            server_pub.publish(goal);
        }        
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
    //int *client_sockfd;
    int server_len;//, client_len;
    struct sockaddr_in server_address;
    //struct sockaddr_in client_address;
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
        int *client_sockfd;
        int client_len;
        struct sockaddr_in client_address;

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
    shutdown(server_sockfd, 2);

    return 0;
}