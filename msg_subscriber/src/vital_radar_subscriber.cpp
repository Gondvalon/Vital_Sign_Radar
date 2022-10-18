#include <ros/ros.h>
#include <vital_sign_msgs/VitalSigns.h>
#include <iostream>
#include <fstream>
#include <cmath>


void chatterCallback(const vital_sign_msgs::VitalSigns::ConstPtr& msg)
{
    std::fstream vitalSignData;
    vitalSignData.open("/home/philipp/Uni/10_SoSe/BachelorSimulation/Data/Threshold_Test/threshold_001.txt", std::fstream::app | std::fstream::in | std::fstream::out);
    if (msg->measurements.size() == 0) {
        vitalSignData << "0 0 0 0 0" << std::endl;
    } else {
        double distance = sqrt(pow(msg->measurements[0].location.x, 2.0) +
                               pow(msg->measurements[0].location.y, 2.0) +
                               pow(msg->measurements[0].location.z, 2.0));
        printf("X:%f Y:%f Z:%f\n", msg->measurements[0].location.x, msg->measurements[0].location.y,msg->measurements[0].location.z);
        printf("Distance: %f\n", distance);

        if (vitalSignData) {
            vitalSignData << "" << distance << " " << msg->measurements[0].heart_rate << " " <<
                   msg->measurements[0].heart_rate_variance << " " <<
                   msg->measurements[0].breathing_rate << " " <<
                   msg->measurements[0].breathing_rate_variance << std::endl;

            vitalSignData.close();
        } else {
            printf("File does not exist \n");
        }
    }
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("vitalRadar", 100, chatterCallback);

  ros::spin();
  return 0;
}