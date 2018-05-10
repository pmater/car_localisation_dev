#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include <iostream>

using namespace std;

int msgCount = 0;

void GPSCallback(const sensor_msgs::NavSatFix &gps);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gpsprinter");
    ros::NodeHandle n;
    
    auto gpsSub = n.subscribe("/gps/fix", 1000, GPSCallback);

    cout << "Spinning" << endl;
    ros::spin();
    return 0;
}

void GPSCallback(const sensor_msgs::NavSatFix &gps)
{
    msgCount++;

    if (msgCount % 10 == 0)
    {
        cout << setprecision(15) << gps.latitude << ", " << gps.longitude << endl;
    }
}