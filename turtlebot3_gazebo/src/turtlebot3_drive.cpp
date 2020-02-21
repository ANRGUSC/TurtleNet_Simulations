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

#include "turtlebot3_gazebo/turtlebot3_drive.h"

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
  // escape_range_       = 30.0 * DEG2RAD;
  escape_range_       = 10.0 * DEG2RAD;

  check_forward_dist_ = 0.7;
  check_side_dist_    = 0.4;
  // check_forward_dist_ = 1.0;
  // check_side_dist_    = 1.0;

  tb3_pose_ = 0.0;
  prev_tb3_pose_ = 0.0;

  // initialize publishers
  cmd_vel_pub_   = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_name, 10);

  // initialize subscribers
  laser_scan_sub_  = nh_.subscribe("scan", 10, &Turtlebot3Drive::laserScanMsgCallBack, this);
  odom_sub_ = nh_.subscribe("odom", 10, &Turtlebot3Drive::odomMsgCallBack, this);

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
  // uint16_t scan_angle[3] = {0, 30, 330};
  uint16_t scan_angle[5] = {0, 10, 350, 30, 330};


  for (int num = 0; num < 5; num++)
  {
    if (std::isinf(msg->ranges.at(scan_angle[num])))
    {
      scan_data_[num] = msg->range_max;
    }
    else if (msg->ranges.at(scan_angle[num]) > 0)
    {
      scan_data_[num] = msg->ranges.at(scan_angle[num]);
    } else {
      ROS_WARN("error reading scan in drive node");
    }
  }
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
    bool leftclose;
    if (scan_data_[LEFT] < check_side_dist_ || scan_data_[FARLEFT] < check_side_dist_) {
        leftclose = true;
    } else {leftclose = false;}

    bool rightclose;
    if (scan_data_[RIGHT] < check_side_dist_ || scan_data_[FARRIGHT] < check_side_dist_) {
        rightclose = true;
    } else {rightclose = false;}

    bool centerclose;
    if (scan_data_[CENTER] < check_forward_dist_) {
        centerclose = true;
    } else {centerclose = false;}

  static uint8_t turtlebot3_state_num = 0;

  switch(turtlebot3_state_num)
  {
    case GET_TB3_DIRECTION:
    // ROS_INFO("%.3f, %.3f %.3f, %.3f, %.3f", scan_data_[FARLEFT], scan_data_[LEFT], scan_data_[CENTER], scan_data_[RIGHT], scan_data_[FARRIGHT]);
      if (!centerclose)
      {
        if (leftclose && !rightclose)
        {
          // ROS_INFO("turn right");
          prev_tb3_pose_ = tb3_pose_;
          turtlebot3_state_num = TB3_RIGHT_TURN;
        }
        else if (rightclose && !leftclose)
        {
          // ROS_INFO("turn left");
          prev_tb3_pose_ = tb3_pose_;
          turtlebot3_state_num = TB3_LEFT_TURN;
        }
        else
        {
          // ROS_INFO("forward");
          turtlebot3_state_num = TB3_DRIVE_FORWARD;
        }
      }

      if (centerclose)
      {
        if (leftclose) {
            prev_tb3_pose_ = tb3_pose_;
            // ROS_INFO("backward (left)");
            turtlebot3_state_num = TB3_DRIVE_BACKWARD_LEFT;
        } else {
            prev_tb3_pose_ = tb3_pose_;
            // ROS_INFO("backward (right)");
            turtlebot3_state_num = TB3_DRIVE_BACKWARD_RIGHT;
        }

      }
      break;


    // case GET_TB3_DIRECTION:
    //   if (scan_data_[CENTER] > check_forward_dist_) {
    //       prev_tb3_pose_ = tb3_pose_;
    //       turtlebot3_state_num = TB3_DRIVE_FORWARD;
    //   } else {
    //       if (scan_data_[LEFT] <= check_side_dist_) {
    //           if (scan_data_[RIGHT] > check_side_dist_) {
    //               prev_tb3_pose_ = tb3_pose_;
    //               turtlebot3_state_num = TB3_RIGHT_TURN;
    //           } else {
    //               prev_tb3_pose_ = tb3_pose_;
    //               turtlebot3_state_num = TB3_DRIVE_BACKWARD;
    //           }
    //       } else {
    //           prev_tb3_pose_ = tb3_pose_;
    //           turtlebot3_state_num = TB3_LEFT_TURN;
    //       }
    //   }
    //   break;

    case TB3_DRIVE_FORWARD:
      updatecommandVelocity(LINEAR_VELOCITY, 0.0);
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;

    case TB3_DRIVE_BACKWARD_LEFT:
      updatecommandVelocity(BACKWARD_LINEAR_VELOCITY, -1*ANGULAR_VELOCITY);
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;

    case TB3_DRIVE_BACKWARD_RIGHT:
      updatecommandVelocity(BACKWARD_LINEAR_VELOCITY, ANGULAR_VELOCITY);
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;

    case TB3_RIGHT_TURN:
      // if (fabs(prev_tb3_pose_ - tb3_pose_) >= escape_range_)
      //   turtlebot3_state_num = GET_TB3_DIRECTION;
      // else
      updatecommandVelocity(0.1, -1*ANGULAR_VELOCITY);
      turtlebot3_state_num = GET_TB3_DIRECTION;
      break;

    case TB3_LEFT_TURN:
      // if (fabs(prev_tb3_pose_ - tb3_pose_) >= escape_range_)
      //   turtlebot3_state_num = GET_TB3_DIRECTION;
      // else
      updatecommandVelocity(0.1, ANGULAR_VELOCITY);
      turtlebot3_state_num = GET_TB3_DIRECTION;
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
  ros::init(argc, argv, "turtlebot3_drive");
  Turtlebot3Drive turtlebot3_drive;

  ros::Rate loop_rate(125);

  // ROS_INFO("get namespace...");
  std::string ns = ros::this_node::getNamespace();
  ROS_INFO("%s",ns.c_str());
  // ROS_INFO("%u",ns.compare("/"));

  // while (ros::ok())
  // {
  //   ros::Time rightnow = ros::Time::now();
  //   // ROS_INFO("%d",rightnow.sec%20);
  //
  //   if (ns.compare("//tb3_0")==0) {
  //       if (rightnow.sec%20==0 || rightnow.sec%20==1 || rightnow.sec%20==2 || rightnow.sec%20==3 || rightnow.sec%20==4) {
  //           // ROS_INFO("tb3_0 drives");
  //           turtlebot3_drive.controlLoop();
  //       } else {turtlebot3_drive.updatecommandVelocity(0.0, 0.0);}}
  //
  //   if (ns.compare("//tb3_1")==0) {
  //       if (rightnow.sec%20==5 || rightnow.sec%20==6 || rightnow.sec%20==7 || rightnow.sec%20==8 || rightnow.sec%20==9) {
  //           // ROS_INFO("tb3_1 drives");
  //           turtlebot3_drive.controlLoop();
  //       } else {turtlebot3_drive.updatecommandVelocity(0.0, 0.0);}}
  //
  //   if (ns.compare("//tb3_2")==0) {
  //       if (rightnow.sec%20==10 || rightnow.sec%20==11 || rightnow.sec%20==12 || rightnow.sec%20==13 || rightnow.sec%20==14) {
  //           // ROS_INFO("tb3_2 drives");
  //           turtlebot3_drive.controlLoop();
  //       } else {turtlebot3_drive.updatecommandVelocity(0.0, 0.0);}}
  //
  //   if (ns.compare("//tb3_3")==0) {
  //       if (rightnow.sec%20==15 || rightnow.sec%20==16 || rightnow.sec%20==17 || rightnow.sec%20==18 || rightnow.sec%20==19) {
  //           // ROS_INFO("tb3_3 drives");
  //           turtlebot3_drive.controlLoop();
  //       } else {turtlebot3_drive.updatecommandVelocity(0.0, 0.0);}}
  //
  //   ros::spinOnce();
  //   loop_rate.sleep();
  //   }

  while (ros::ok())
  {
    turtlebot3_drive.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
    }

  return 0;
}
