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
#include "std_msgs/Bool.h"
ros::Publisher command_pub;
ros::Publisher aeb_state_pub;
ackermann_msgs::AckermannDriveStamped drive_msg;

const double current_speed = 0.5;
const double brake_dis = 1.0; 

bool reversing = false; 
ros::Time reverse_start_time; 
const ros::Duration reverse_duration(5.0);

float speed_limit = 1.8;
float angle_limit = 0.3;

void backtosafe(ackermann_msgs::AckermannDriveStamped& drive_msg, std_msgs::Bool& aeb_state, double min_ttc) {
  double safe_dis = brake_dis;
  if (reversing) {
      if (min_ttc > safe_dis) {
          reversing = false;
          drive_msg.drive.speed = 0; 
          aeb_state.data = false; 
      } else {
          drive_msg.drive.speed = -2; 
      }
  } else if (aeb_state.data && drive_msg.drive.speed == 0 && !reversing) {
      if (min_ttc <= safe_dis) {
          reversing = true;
          drive_msg.drive.speed = -2; 
      }
  }
}
void AEB(const sensor_msgs::LaserScan& laserscan){
  double min_ttc = std::numeric_limits<double>::infinity();
  
  std_msgs::Bool aeb_state;
  aeb_state.data = false;

  for (size_t i = 0; i < laserscan.ranges.size(); ++i) {
    double angle = (laserscan.angle_min + i * laserscan.angle_increment);
    double speed = current_speed * std::cos(angle);
    double dis = laserscan.ranges[i];
    
    //std::cout<<"speed"<<speed<<std::endl;
    if(std::fabs(speed) < std::numeric_limits<double>::epsilon()){
      continue;
    }
    double ttc = dis / speed;
    if (ttc >= 0 && ttc < min_ttc) {
      min_ttc = ttc;
    }
    //std::cout<<"min_ttc2"<<min_ttc<<std::endl;
  }

  if (min_ttc < brake_dis){
    aeb_state.data = true;
    drive_msg.drive.speed = 0; 
    //std::cout<<"brake"<<std::endl;
  }else{
    aeb_state.data = false;
    drive_msg.drive.speed = current_speed; 
    //std::cout<<"using current"<<std::endl;
  }
  backtosafe(drive_msg, aeb_state, min_ttc);
  command_pub.publish(drive_msg);
  aeb_state_pub.publish(aeb_state);
}

std_msgs::String stdStringToRosString(std::string message) {
  std_msgs::String msg;
  msg.data = message;
  return msg;
}
sensor_msgs::LaserScan scans;
void LaserScanCallback(const sensor_msgs::LaserScan::ConstPtr& laserscan){
  scans = *laserscan;
  AEB(*laserscan);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "aeb");
  ros::NodeHandle n;

  command_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/aeb_command", 1);
  ros::Subscriber scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, LaserScanCallback);
  aeb_state_pub = n.advertise<std_msgs::Bool>("/aeb_state", 1, true); 
  
  // Make and publish message
  //  Header
  std_msgs::Header header;
  header.stamp = ros::Time::now();
  drive_msg.drive.steering_angle = 0.0;
  //  AckermannStamped

  drive_msg.header = header;
  ros::spin();

  return 0;
}