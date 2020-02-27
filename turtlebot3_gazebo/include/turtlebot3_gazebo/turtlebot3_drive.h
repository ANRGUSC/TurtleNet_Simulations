/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#ifndef TURTLEBOT3_DRIVE_H_
#define TURTLEBOT3_DRIVE_H_

#include <ros/ros.h>

#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#define DEG2RAD (M_PI / 180.0)
#define RAD2DEG (180.0 / M_PI)

#define CENTER 0
#define LEFT   1
#define RIGHT  2
#define FARLEFT 3
#define FARRIGHT 4

// #define LINEAR_VELOCITY  0.3
// #define ANGULAR_VELOCITY 1.5
#define LINEAR_VELOCITY  0.3
#define BACKWARD_LINEAR_VELOCITY -0.2

// #define LINEAR_VELOCITY  0.1
// #define BACKWARD_LINEAR_VELOCITY -0.1

#define ANGULAR_VELOCITY 0.4

#define MOVE_FORWARD  0
#define STOP          1

class Turtlebot3Drive
{
 public:
  Turtlebot3Drive();
  ~Turtlebot3Drive();
  bool init();
  bool controlLoop();
  void updatecommandVelocity(double linear, double angular);

 private:
  // ROS NodeHandle
  ros::NodeHandle nh_;
  ros::NodeHandle nh_priv_;

  // ROS Parameters

  // ROS Time

  // ROS Topic Publishers
  ros::Publisher cmd_vel_pub_;

  // ROS Topic Subscribers
  ros::Subscriber laser_scan_sub_;
  ros::Subscriber odom_sub_;

  // Variables
  double escape_range_;
  double check_forward_dist_;
  double check_side_dist_;

  double scan_data_[5] = {0.0, 0.0, 0.0, 0.0, 0.0};

  double tb3_pose_;
  double prev_tb3_pose_;

  // Function prototypes
  void laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg);
  void odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg);
};
#endif // TURTLEBOT3_DRIVE_H_
