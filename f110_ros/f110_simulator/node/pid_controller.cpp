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
#include <sensor_msgs/LaserScan.h>
#include <math.h>
float speed;
float angle;
float speed_limit = 1.8;
float angle_limit = 0.3;
float min_steer =-0.3;
float max_steer =0.3;
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

double pd_controller(const sensor_msgs::LaserScan& laserscan){
  float k_p = -2.0;
  float k_d = 0.5;
  float k_i = 0.1;
  float goal_dis = 1.5;
  float u_t = 0;
  float err;
  float pre_err;
  float accum_err = 0;
  // get the min dis.
  auto minit = std::min_element(laserscan.ranges.begin(),laserscan.ranges.end());
  if (minit != laserscan.ranges.end()){
    int min_index_dis = std::distance(laserscan.ranges.begin(),minit);
    angle = (laserscan.angle_min + min_index_dis * laserscan.angle_increment);
  }
  float dis_car_wall = *minit;
  err = goal_dis - (abs(abs(dis_car_wall) + speed * sin(angle - 1)));
  accum_err += err;
  pre_err = err;
  u_t = err* k_p + (pre_err - err) * k_d+ accum_err * k_i;
  //if(dis_car_wall < goal_dis){
    //err = goal_dis - (abs(abs(dis_car_wall) + speed * sin(angle)));
    //u_t = err* k_p;
  //}
  

  float new_angle = std::max(min_steer, std::min(max_steer, u_t));

  std::cout<<"new_angle:"<<new_angle<<std::endl;
  std::cout<<"*minit:"<<*minit<<std::endl;
  std::cout<<"angle:"<<angle<<std::endl;
  std::cout<<"err:"<<err<<std::endl;
  return new_angle;
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "ftg");
  ros::NodeHandle n;
  //ros::Publisher command_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);
  ros::Publisher command_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>("/drive", 1);
  ros::Subscriber scan_sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, LaserScanCallback);

  ros::Rate loop_rate(10);


  while(ros::ok()) {
    ros::spinOnce();
    if(getthedata){
      speed = 1.0;
      angle = pd_controller(scans);
      std::cout<<"act_angle"<<angle;
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
