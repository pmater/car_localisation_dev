#include "ros/ros.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"
#include <iostream>

using namespace std;

ros::Publisher odomPub;

void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg);

int main(int argc, char **argv) //Mapviz can't display PoseWithCovarianceStamped, so we convert it to odometry
{
    ros::init(argc, argv, "pose_to_odometry");
    ros::NodeHandle n;
    
    auto poseSub = n.subscribe("/integrated_to_map", 1000, PoseCallback);
    odomPub = n.advertise<nav_msgs::Odometry>("/mapviz/integrated_to_map", 1000);

    cout << "Spinning" << endl;
    ros::spin();
    return 0;
}

void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
    nav_msgs::Odometry odomMsg;
    odomMsg.header = msg.header;
    odomMsg.pose = msg.pose;

    odomPub.publish(odomMsg);
}