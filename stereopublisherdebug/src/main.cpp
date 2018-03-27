#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

const string userFolder = "ubuntu";

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereopublisherdebug");
	ros::NodeHandle n;

    auto leftPub = n.advertise<sensor_msgs::Image>("/camera/left/image_raw", 1000);
    auto rightPub = n.advertise<sensor_msgs::Image>("/camera/right/image_raw", 1000);


    string folder = argv[1];
	cout << "Image folder: " << folder << endl;

    ifstream timeFile("/home/" + userFolder + "/slam/" + folder + "/times.txt");
    string timeLine;

    int camPublishFrequency;
    n.getParam("camPublishFrequency", camPublishFrequency);
    ros::Rate loop_rate(camPublishFrequency); //frequency (hz)
    int i = 0;
    while (ros::ok())
    {
        Mat imageLeft = imread("/home/" + userFolder + "/slam/" + folder + "/image_0/" + to_string(i) + ".png", CV_LOAD_IMAGE_COLOR);
        Mat imageRight = imread("/home/" + userFolder + "/slam/" + folder + "/image_1/" + to_string(i) + ".png", CV_LOAD_IMAGE_COLOR);

        if (imageLeft.empty())
        {
            break;
        }

        getline(timeFile, timeLine);
        double timeD = stod(timeLine);
        ros::Time timeROS(timeD);

        std_msgs::Header header;
		header.seq = i;
		header.stamp = timeROS;
        auto bridgeLeft = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, imageLeft);
        auto bridgeRight = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, imageRight);

		sensor_msgs::Image imgMsgLeft, imgMsgRight;
        bridgeLeft.toImageMsg(imgMsgLeft);
        bridgeRight.toImageMsg(imgMsgRight);

		leftPub.publish(imgMsgLeft);
        rightPub.publish(imgMsgRight);

        i++;
        ros::spinOnce();
		loop_rate.sleep();
    }

    cout << "Image publisher done" << endl;
    timeFile.close();
    ros::shutdown();
    return 0;
}
