#include "ros/ros.h"
#include "std_msgs/String.h"
#include <ackermann_msgs/AckermannDriveStamped.h>
std::string mode_msg;
ros::Publisher command_pub;

void mode_callback(const std_msgs::String::ConstPtr& msg) {
    mode_msg = msg->data;
    ROS_INFO("Recv: %s", msg->data.c_str());
}
void keyboard_callback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
    if(mode_msg == "k"){
        command_pub.publish(msg);
    }
    //ROS_INFO("Recv: %s", msg->data.c_str());
}

void ftg_callback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg) {
    if(mode_msg== "f"){
        //ROS_INFO("Recv: %s", msg->data.c_str());
        command_pub.publish(msg);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "mux");
    ros::NodeHandle node;

    ros::Subscriber mode_sub = node.subscribe("mode",10,mode_callback);  
    ros::Subscriber ftg_sub = node.subscribe("ftg_mode",10,ftg_callback);
    ros::Subscriber keyboard_sub = node.subscribe("keyborad_mode",10,keyboard_callback);
    command_pub = node.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 10);
    //ros::Rate loop_rate(5);

    ros::spin();
    //loop_rate.sleep();
    return 0;
}