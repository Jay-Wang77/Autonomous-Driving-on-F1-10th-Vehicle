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
#include <math.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/LaserScan.h>


float speed_limit = 1.8;
float angle_limit = 0.3;

std_msgs::String stdStringToRosString(std::string message) {
  std_msgs::String msg;
  msg.data = message;
  return msg;
}
sensor_msgs::LaserScan scans;
bool getthedata = false;
void LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserscan){
  scans = *laserscan;
  getthedata = true;
}

double FollowTheGap(const sensor_msgs::LaserScan& laserscan){
  auto minit = std::min_element(laserscan.ranges.begin(),laserscan.ranges.end());
  double angle;
  if (minit != laserscan.ranges.end() && *minit < 0.55){
    int min_index_dis = std::distance(laserscan.ranges.begin(),minit);
    angle = (laserscan.angle_min + min_index_dis * laserscan.angle_increment);
    angle +=  M_PI;
  }else{
    auto maxit = std::max_element(laserscan.ranges.begin()+270,laserscan.ranges.end()-270);
    if (maxit != laserscan.ranges.end()){
      int max_index_dis = std::distance(laserscan.ranges.begin(),maxit);
      angle = laserscan.angle_min + max_index_dis * laserscan.angle_increment;
  }else{
      return 0.0;
  }
  }
  while(angle>M_PI){
    angle -= 2*M_PI;
  }
  while(angle<-M_PI){
    angle += 2*M_PI;
  }
  return angle;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ftg");
  ros::NodeHandle n;
  //ros::Publisher command_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);
  ros::Publisher command_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("ftg_mode", 1);
  ros::Subscriber scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, LaserScanCallback);
  ros::Rate loop_rate(10);
  double speed;
  double angle;

  while(ros::ok()) {
    ros::spinOnce();
    if(getthedata){
      speed = 1.0;
      angle = FollowTheGap(scans);
    }
    getthedata = false;
    // Make and publish message
    //  Header
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    //  Ackermann
    ackermann_msgs::AckermannDrive drive_msg;
    drive_msg.speed = speed * speed_limit;
    drive_msg.steering_angle = angle ;
    //  AckermannStamped
    ackermann_msgs::AckermannDriveStamped drive_st_msg;
    drive_st_msg.header = header;
    drive_st_msg.drive = drive_msg;
    // publish AckermannDriveStamped message to drive topic
    command_pub.publish(drive_st_msg);
    loop_rate.sleep();
  }
  return 0;
}
