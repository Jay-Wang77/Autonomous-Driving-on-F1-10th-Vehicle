#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include <ackermann_msgs/AckermannDriveStamped.h>
ros::Publisher command_pub;
bool aeb = false;

void keyboard_callback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
    if(!aeb){
        command_pub.publish(msg);
    }
}


void aeb_contr_callback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
    if(aeb){
        command_pub.publish(msg);
    }
}

void aeb_state_callback(const std_msgs::Bool::ConstPtr& msg) {
    aeb = msg->data;
    //ROS_INFO("state: %s", aeb);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mux");
    ros::NodeHandle node;

    command_pub = node.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 10);
    ros::Subscriber keyboard_sub = node.subscribe("/keyboard_mode",10,keyboard_callback);
    ros::Subscriber aeb_contr_sub = node.subscribe("/aeb_command",10,aeb_contr_callback);
    ros::Subscriber aeb_state_sub = node.subscribe("/aeb_state",10,aeb_state_callback);

    ros::spin();
    return 0;
}