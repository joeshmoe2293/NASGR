#include <stdio.h>
#include <sstream>
#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/String.h"

void poseCallback(const geometry_msgs::Pose& sum_msg);

ros::Publisher pub;
ros::Publisher str_pub;
ros::Subscriber sub;

int main(int argc, char** argv) {
    ros::init(argc, argv, "posePublisher");

    ros::NodeHandle nh;
    pub = nh.advertise<geometry_msgs::Pose>("poses", 1);
    sub = nh.subscribe("sum", 1, poseCallback);
    str_pub = nh.advertise<std_msgs::String>("sum_messages", 1);
    ros::Rate rate(1);

    while (ros::ok()) {
        geometry_msgs::Pose pose;

        pose.position.x = 1;
        pose.position.y = 2;
        pose.position.z = 3;
        pose.orientation.x = 0;
        pose.orientation.y = 0;
        pose.orientation.z = 0;
        pose.orientation.w = 1;

        pub.publish(pose);
        ROS_INFO("Published pose");
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

void poseCallback(const geometry_msgs::Pose& sum_msg) {
    std::stringstream msg;
    msg << "I heard a pose: \n";
    msg << "Position X: " << sum_msg.position.x << "\n";
    msg << "Position Y: " << sum_msg.position.y << "\n";
    msg << "Position Z: " << sum_msg.position.z << "\n";
    msg << "Orientation X: " << sum_msg.orientation.x << "\n";
    msg << "Orientation Y: " << sum_msg.orientation.y << "\n";
    msg << "Orientation Z: " << sum_msg.orientation.z << "\n";
    msg << "Orientation W: " << sum_msg.orientation.w << "\n";

    std_msgs::String message;
    message.data = msg.str();

    str_pub.publish(message);
}
