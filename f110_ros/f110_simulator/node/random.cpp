#define _GNU_SOURCE

#include <iostream>
#include <string>
#include <sstream>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <cstdlib>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

//                       0:       1:       2:      3:
float mapping[5][2] = {{0.5, 0.0}, {0.5, -0.5}, {0.5, 0.5}, {-0.5, 0.0}, {0.0, 0.0}};

float speed_limit = 1.8;
float angle_limit = 0.3;

std_msgs::String stdStringToRosString(std::string message) {
  std_msgs::String msg;
  msg.data = message;
  return msg;
}

unsigned keyToIndex(char message) {
  unsigned res;
  if (message == 'w') {
    res = 0;
  }
  else if (message == 'd') {
    res = 1;
  }
  else if (message == 'a') {
    res = 2;
  }
  else if (message == 's') {
    res = 3;
  }
  else if (message == ' ') {
    res = 4;
  }
  else {
    res = 4;
  }
  return res;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "random_teleop");

  ros::NodeHandle n;

  ros::Publisher command_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);
  ros::Rate loop_rate(10);
  float speed = 0.0;
  float angle = 0.0;

  while(ros::ok()) {
    int random_index;
    random_index = rand() % 4;
    speed = mapping[random_index][0];
    angle = mapping[random_index][1];

    // Make and publish message
    //  Header
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    //  Ackermann
    ackermann_msgs::AckermannDrive drive_msg;
    drive_msg.speed = speed * speed_limit;
    drive_msg.steering_angle = angle * angle_limit;
    //  AckermannStamped
    ackermann_msgs::AckermannDriveStamped drive_st_msg;
    drive_st_msg.header = header;
    drive_st_msg.drive = drive_msg;
    // publish AckermannDriveStamped message to drive topic
    command_pub.publish(drive_st_msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
