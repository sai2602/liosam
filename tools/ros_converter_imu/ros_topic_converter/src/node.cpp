#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"

#include <sstream>

//#include <stdlib.h>

ros::Subscriber sub_gen;
ros::Publisher pub_gen;

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  sensor_msgs::Imu imu_msg(*msg);
  /*
  imu_msg.header = msg->header;
  imu_msg.orientation = msg.orientation;
  imu_msg.angular_velocity = msg.angular_velocity;
  imu_msg.linear_acceleration = msg.linear_acceleration;
  */
  imu_msg.header.frame_id = "imu";
  pub_gen.publish(imu_msg);
  std::cout << ".";
}

void odomCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
  nav_msgs::Odometry odom_msg;
  odom_msg.header = msg->header;
  odom_msg.header.frame_id = "odom";
  odom_msg.child_frame_id = "odom";
  odom_msg.pose.pose.orientation.x = 0;
  odom_msg.pose.pose.orientation.y = 0;
  odom_msg.pose.pose.orientation.z = 0;
  odom_msg.pose.pose.orientation.w = 1;
  odom_msg.twist.twist = msg->twist;
  pub_gen.publish(odom_msg);
  std::cout << ".";
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ros_topic_converter");

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string setting;
  nh_private.param("setting", setting, std::string(""));
  std::cout << "settting: " << setting << std::endl;

  if (!strcmp(setting.c_str(),"convert_odom")) {
      ROS_INFO("Convert odom topic: /abss/twist --> /odom");
      sub_gen = nh.subscribe("/abss/twist", 1000, odomCallback);
      pub_gen = nh.advertise<nav_msgs::Odometry>("odom", 1000);
  } else if (!strcmp(setting.c_str(),"convert_imu")) {
      ROS_INFO("Convert imu topic: /imu0 --> /imu");
      sub_gen = nh.subscribe("/imu0", 1000, imuCallback);
      pub_gen = nh.advertise<sensor_msgs::Imu>("imu", 1000);
  } else {
     ROS_INFO("Unknown setting.");
     return 0;
  }

  ros::spin();

  return 0;
}
