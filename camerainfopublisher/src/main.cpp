#include "ros/ros.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/Image.h"
#include <iostream>

using namespace std;

void LeftCallback(const sensor_msgs::ImageConstPtr &msg);
void RightCallback(const sensor_msgs::ImageConstPtr &msg);

sensor_msgs::CameraInfo leftInfo, rightInfo;
ros::Publisher leftInfoPub, rightInfoPub;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "camerainfopublisher");
    ros::NodeHandle n;
    
    auto leftSub = n.subscribe("/zed/left/image_rect_color", 1000, LeftCallback);
    auto rightSub = n.subscribe("/zed/right/image_rect_color", 1000, RightCallback);
    leftInfoPub = n.advertise<sensor_msgs::CameraInfo>("/zed/left/camera_info", 1000);
    rightInfoPub = n.advertise<sensor_msgs::CameraInfo>("/zed/right/camera_info", 1000);

    leftInfo.height = 720;
    leftInfo.width = 1280;
    leftInfo.distortion_model = "plumb_bob";
    leftInfo.D = {0, 0, 0, 0, 0};
    leftInfo.K = {700.166, 0.0, 641.877, 0.0, 700.166, 395.353, 0.0, 0.0, 1.0};
    leftInfo.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    leftInfo.P = {700.166, 0.0, 641.877, 84.0199*1000, 0.0, 700.166, 395.353, 0.0, 0.0, 0.0, 1.0, 0.0};

    rightInfo.height = 720;
    rightInfo.width = 1280;
    rightInfo.distortion_model = "plumb_bob";
    rightInfo.D = {0, 0, 0, 0, 0};
    rightInfo.K = {699.887, 0.0, 646.385, 0.0, 699.887, 395.712, 0.0, 0.0, 1.0};
    rightInfo.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    rightInfo.P = {699.887, 0.0, 646.385, 83.9864*1000, 0.0, 699.887, 395.712, 0.0, 0.0, 0.0, 1.0, 0.0};   

    cout << "Spinning" << endl;
    ros::spin();
    return 0;
}

void LeftCallback(const sensor_msgs::ImageConstPtr &msg)
{
    leftInfo.header = msg->header;
    leftInfoPub.publish(leftInfo);
}

void RightCallback(const sensor_msgs::ImageConstPtr &msg)
{
    rightInfo.header = msg->header;
    rightInfoPub.publish(rightInfo);
}