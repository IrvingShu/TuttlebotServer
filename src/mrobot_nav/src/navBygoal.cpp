#include "ros/ros.h"
#include "std_msgs/String.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

MoveBaseClient *ac;
ros::Publisher robot_state_pub;

void arriveGoalCallback(const move_base_msgs::MoveBaseGoal &goal)
{
    ROS_INFO("Sending goal");
    ac->sendGoal(goal);
    ac->waitForResult();
    if(ac->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("arrive goal!");
        //发送状态到达了终点
         robot_state_pub.publish("1");

    }
    else
    {
        ROS_INFO("arrive fail!"); 
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "simple_navigation_goals");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("server", 1000, arriveGoalCallback);
    robot_state_pub = n.advertise<std_msgs::String>("robot_state", 1000);


    ac = new MoveBaseClient("move_base", true);
    while(!ac->waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    ros::spin();
    ROS_INFO("spin end!");
    return 0;
}