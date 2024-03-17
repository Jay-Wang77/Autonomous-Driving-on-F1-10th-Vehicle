#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ackermann_msgs/AckermannDriveStamped.h>
std::string mode_msg;
ros::Publisher command_pub;

void mode_callback(const std_msgs::String::ConstPtr& msg) {
    mode_msg = msg->data;
    
}
void keyboard_callback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
    if(mode_msg == "k"){
        command_pub.publish(msg);
    }
}

void pid_callback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
    if(mode_msg== "p"){
        //ROS_INFO("Recv: %s", mode_msg);
        command_pub.publish(msg);
    }
}
void ftg_callback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
    if(mode_msg== "f" ){
        //ROS_INFO("Recv: %s", msg->data.c_str());
        command_pub.publish(msg);
    }
}
/***void js_callback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
    if(mode_msg== "j"){
        ROS_INFO("Recv: %s", mode_msg);
        command_pub.publish(msg);
    }
}***/

int main(int argc, char **argv) {
    ros::init(argc, argv, "mux2"); 
    ros::NodeHandle node;

    ros::Subscriber mode_sub = node.subscribe("/mode",10,mode_callback);
    ros::Subscriber pid_sub = node.subscribe("/pid",10,pid_callback);
    ros::Subscriber keyboard_sub = node.subscribe("/keyborad_mode",10,keyboard_callback);
    //ros::Subscriber js_sub = node.subscribe("/vesc/js",10,js_callback);
    //command_pub = node.advertise<ackermann_msgs::AckermannDriveStamped>("/vesc/low_level/ackermann_cmd_mux/input/teleop", 10);
    ros::Subscriber ftg_sub = node.subscribe("/ftg_mode",10,ftg_callback);
    command_pub = node.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 10);
    
    ros::spin();
    return 0;
}
