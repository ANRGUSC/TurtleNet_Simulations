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

#include "turtlebot3_gazebo/new_drive.h"

Turtlebot3Drive::Turtlebot3Drive()
  : nh_priv_("~")
{
  //Init gazebo ros turtlebot3 node
  ROS_INFO("TurtleBot3 Simulation Node Init");
  ROS_ASSERT(init());
}

Turtlebot3Drive::~Turtlebot3Drive()
{
  updatecommandVelocity(0.0, 0.0);
  ros::shutdown();
}

/*******************************************************************************
* Init function
*******************************************************************************/
bool Turtlebot3Drive::init()
{
  // initialize ROS parameter
  std::string cmd_vel_topic_name = nh_.param<std::string>("cmd_vel_topic_name", "");

  // initialize variables
  escape_range_       = 5.0 * DEG2RAD;
  check_forward_dist_ = 0.8;
  check_side_dist_    = 0.25;

  tb3_pose_ = 0.0;
  prev_tb3_pose_ = 0.0;

  // initialize publishers
  // cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);
  cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 1);

  // initialize subscribers
  laser_scan_sub_  = nh_.subscribe("scan", 10, &Turtlebot3Drive::laserScanMsgCallBack, this);
  odom_sub_ = nh_.subscribe("odom", 10, &Turtlebot3Drive::odomMsgCallBack, this);

  got_first_scan = false;

  return true;
}

void Turtlebot3Drive::odomMsgCallBack(const nav_msgs::Odometry::ConstPtr &msg)
{
  double siny = 2.0 * (msg->pose.pose.orientation.w * msg->pose.pose.orientation.z + msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
	double cosy = 1.0 - 2.0 * (msg->pose.pose.orientation.y * msg->pose.pose.orientation.y + msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);

	tb3_pose_ = atan2(siny, cosy);
}

void Turtlebot3Drive::laserScanMsgCallBack(const sensor_msgs::LaserScan::ConstPtr &msg)
{
  // uint16_t scan_angle[5] = {330, 353, 0, 7, 30};
  uint16_t scan_angle[5] = {30, 7, 0, 353, 330};

  for (int num = 0; num < 5; num++)
  {
    if (std::isinf(msg->ranges.at(scan_angle[num])))
    {
      scan_data_[num] = msg->range_max;
    }
    else if (msg->ranges.at(scan_angle[num]) == 0)
    {
      scan_data_[num] = msg->range_max;
    }
    else {
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    }
  }
  got_first_scan = true;
  ROS_INFO("%.3f, %.3f, %.3f, %.3f, %.3f",scan_data_[LEFT],scan_data_[L], scan_data_[CENTER],scan_data_[R],scan_data_[RIGHT]);
}

void Turtlebot3Drive::updatecommandVelocity(double linear, double angular)
{
  geometry_msgs::Twist cmd_vel;

  cmd_vel.linear.x  = linear;
  cmd_vel.angular.z = angular;

  cmd_vel_pub_.publish(cmd_vel);
}

/*******************************************************************************
* Control Loop function
*******************************************************************************/
bool Turtlebot3Drive::controlLoop()
{
  static uint8_t turtlebot3_state_num = 0;

  switch(turtlebot3_state_num)
  {
    case GET_TB3_DIRECTION:

      if (!got_first_scan){
          ROS_INFO("WAITNG FOR SCAN");
          turtlebot3_state_num = 0;
      }

      else if (scan_data_[CENTER] < check_forward_dist_ && scan_data_[L] < check_forward_dist_ && scan_data_[R] < check_forward_dist_) {
          ROS_INFO("MY PATH IS ALL BLOCKED");
          escape_range_ = 90 * DEG2RAD;
          if (scan_data_[L] < scan_data_[R])
          {
            ROS_INFO("TURN RIGHT");
            prev_tb3_pose_ = tb3_pose_;
            turtlebot3_state_num = TB3_RIGHT_TURN;
          }
          else
          {
            ROS_INFO("TURN LEFT");
            prev_tb3_pose_ = tb3_pose_;
            turtlebot3_state_num = TB3_LEFT_TURN;
          }
      }

      else {
          escape_range_ = 5.0 * DEG2RAD;
          if (scan_data_[CENTER] > check_forward_dist_ && scan_data_[L] > check_forward_dist_ && scan_data_[R] > check_forward_dist_ && scan_data_[LEFT] > check_side_dist_ && scan_data_[RIGHT] > check_side_dist_)
          // if (scan_data_[CENTER] > check_forward_dist_)
          {
            ROS_INFO("NOTHING TOO CLOSE");
            turtlebot3_state_num = TB3_DRIVE_FORWARD;
          }

          // if (scan_data_[CENTER] < check_forward_dist_ || scan_data_[L] < check_forward_dist_)
          // if (scan_data_[CENTER] < check_forward_dist_)
          // else if (scan_data_[L] < check_side_dist_ || scan_data_[LEFT] < check_side_dist_)
          else if (scan_data_[L] < scan_data_[R] || scan_data_[LEFT] < check_side_dist_)
          {
            ROS_INFO("My slight left has a closer obstacle");
            prev_tb3_pose_ = tb3_pose_;
            turtlebot3_state_num = TB3_RIGHT_TURN;
          }

          // else if (scan_data_[CENTER] < check_forward_dist_)
          // else if (scan_data_[CENTER] < check_forward_dist_ || scan_data_[R] < check_forward_dist_)
          else
          {
            ROS_INFO("My slight right has a closer obstacle");
            prev_tb3_pose_ = tb3_pose_;
            turtlebot3_state_num = TB3_LEFT_TURN;
          }
      }
      break;

    case TB3_DRIVE_FORWARD:
      ROS_DEBUG("DRIVE FORWARD");
      updatecommandVelocity(LINEAR_VELOCITY, 0.0);
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;

    case TB3_RIGHT_TURN:
      ROS_DEBUG("TURN RIGHT");
      if (fabs(prev_tb3_pose_ - tb3_pose_) >= escape_range_)
        turtlebot3_state_num = GET_TB3_DIRECTION;
      else
        // updatecommandVelocity(0.0, -1 * ANGULAR_VELOCITY);
        updatecommandVelocity(0.0, -1 * ANGULAR_VELOCITY);
      break;

    case TB3_LEFT_TURN:
      ROS_DEBUG("TURN LEFT");
      if (fabs(prev_tb3_pose_ - tb3_pose_) >= escape_range_)
        turtlebot3_state_num = GET_TB3_DIRECTION;
      else
        // updatecommandVelocity(0.0, ANGULAR_VELOCITY);
        updatecommandVelocity(0.0, ANGULAR_VELOCITY);
      break;

    default:
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;
  }

  return true;
}
/*******************************************************************************
* Main function
*******************************************************************************/
int main(int argc, char* argv[])
{
  ros::init(argc, argv, "new_drive");
  Turtlebot3Drive turtlebot3_drive;

  ros::Rate loop_rate(125);
  // ROS_INFO("get namespace...");
  std::string ns = ros::this_node::getNamespace();
  ROS_INFO("%s",ns.c_str());
  // ROS_INFO("%u",ns.compare("/"));

  while (ros::ok())
  {
    ros::Time rightnow = ros::Time::now();
    // ROS_INFO("%d",rightnow.sec%20);

    if (ns.compare("//tb3_0")==0) {
        if (rightnow.sec%20==0 || rightnow.sec%20==1 || rightnow.sec%20==2 || rightnow.sec%20==3 || rightnow.sec%20==4) {
            // ROS_INFO("tb3_0 drives");
            turtlebot3_drive.controlLoop();
        } else {turtlebot3_drive.updatecommandVelocity(0.0, 0.0);}}

    if (ns.compare("//tb3_1")==0) {
        if (rightnow.sec%20==5 || rightnow.sec%20==6 || rightnow.sec%20==7 || rightnow.sec%20==8 || rightnow.sec%20==9) {
            // ROS_INFO("tb3_1 drives");
            turtlebot3_drive.controlLoop();
        } else {turtlebot3_drive.updatecommandVelocity(0.0, 0.0);}}

    if (ns.compare("//tb3_2")==0) {
        if (rightnow.sec%20==10 || rightnow.sec%20==11 || rightnow.sec%20==12 || rightnow.sec%20==13 || rightnow.sec%20==14) {
            // ROS_INFO("tb3_2 drives");
            turtlebot3_drive.controlLoop();
        } else {turtlebot3_drive.updatecommandVelocity(0.0, 0.0);}}

    if (ns.compare("//tb3_3")==0) {
        if (rightnow.sec%20==15 || rightnow.sec%20==16 || rightnow.sec%20==17 || rightnow.sec%20==18 || rightnow.sec%20==19) {
            // ROS_INFO("tb3_3 drives");
            turtlebot3_drive.controlLoop();
        } else {turtlebot3_drive.updatecommandVelocity(0.0, 0.0);}}

    ros::spinOnce();
    loop_rate.sleep();
    }

  turtlebot3_drive.updatecommandVelocity(0.0, 0.0);

  return 0;
}
