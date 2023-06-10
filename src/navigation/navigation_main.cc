//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation_main.cc
\brief   Main entry point for reference Navigation implementation
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <signal.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <vector>

#include "glog/logging.h"
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/Localization2DMsg.h"
#include "gflags/gflags.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "sensor_msgs/LaserScan.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"

#include "std_msgs/String.h"

#include "navigation.h"

#include <darknet_ros_msgs/BoundingBoxes.h>

using amrl_msgs::Localization2DMsg;
using math_util::DegToRad;
using math_util::RadToDeg;
using navigation::Navigation;
using ros::Time;
using ros_helpers::Eigen3DToRosPoint;
using ros_helpers::Eigen2DToRosPoint;
using ros_helpers::RosPoint;
using ros_helpers::SetRosVector;
using std::string;
using std::vector;
using Eigen::Vector2f;

// Create command line arguments
DEFINE_string(laser_topic, "scan", "Name of ROS topic for LIDAR data");
DEFINE_string(odom_topic, "odom", "Name of ROS topic for odometry data");
DEFINE_string(loc_topic, "localization", "Name of ROS topic for localization");
DEFINE_string(init_topic,
              "initialpose",
              "Name of ROS topic for initialization");
DEFINE_string(map, "GDC1", "Name of vector map file");

bool run_ = true;
sensor_msgs::LaserScan last_laser_msg_;
Navigation* navigation_ = nullptr;

// TODO: Test
void LaserCallback(const sensor_msgs::LaserScan& msg) {
  if (FLAGS_v > 0) {
    printf("Laser t=%f, dt=%f\n",
           msg.header.stamp.toSec(),
           GetWallTime() - msg.header.stamp.toSec());
  }
  // Location of the laser on the robot. Assumes the laser is forward-facing.
  const Vector2f kLaserLoc(0.2, 0);

  static vector<Vector2f> point_cloud_;
  // TODO Convert the LaserScan to a point cloud
  // The LaserScan parameters are accessible as follows:
  // msg.angle_increment // Angular increment between subsequent rays
  // msg.angle_max // Angle of the first ray
  // msg.angle_min // Angle of the last ray
  // msg.range_max // Maximum observable range
  // msg.range_min // Minimum observable range
  // msg.ranges[i] // The range of the i'th ray

  point_cloud_.clear();

  int index = 0;
  for(double angle = msg.angle_min; angle <= msg.angle_max; angle += msg.angle_increment) {
    double distance = msg.ranges[index];

    // Calculate in "laser" frame
    double point_x = distance * cos(angle);
    double point_y = distance * sin(angle);
   
    // Convert to "base_link" frame
    point_x += kLaserLoc[0];
    point_y += kLaserLoc[1];

    // Add new point into the point cloud
    Vector2f point(point_x, point_y);
    point_cloud_.push_back(point);

    index++;
  }
  navigation_->ObservePointCloud(point_cloud_, msg.header.stamp.toSec());
  last_laser_msg_ = msg;
}

void OdometryCallback(const nav_msgs::Odometry& msg) {
  if (FLAGS_v > 0) {
    printf("Odometry t=%f\n", msg.header.stamp.toSec());
  }
  navigation_->UpdateOdometry(
      Vector2f(msg.pose.pose.position.x, msg.pose.pose.position.y),
      2.0 * atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
      Vector2f(msg.twist.twist.linear.x, msg.twist.twist.linear.y),
      msg.twist.twist.angular.z);
}

void GoToCallback(const geometry_msgs::PoseStamped& msg) {
  const Vector2f loc(msg.pose.position.x, msg.pose.position.y);
  const float angle =
      2.0 * atan2(msg.pose.orientation.z, msg.pose.orientation.w);
  printf("Goal: (%f,%f) %f\u00b0\n", loc.x(), loc.y(), angle);
  navigation_->SetNavGoal(loc, angle);
}

void SignalHandler(int) {
  if (!run_) {
    printf("Force Exit.\n");
    exit(0);
  }
  printf("Exiting.\n");
  run_ = false;
}

void LocalizationCallback(const amrl_msgs::Localization2DMsg msg) {
  if (FLAGS_v > 0) {
    printf("Localization t=%f\n", GetWallTime());
  }
  navigation_->UpdateLocation(Vector2f(msg.pose.x, msg.pose.y), msg.pose.theta);
}

void darknetCallback(const darknet_ros_msgs::BoundingBoxes &bb_msg) {

  int size = bb_msg.bounding_boxes.size(); // ros msgs are mapped onto std::vector thats we can use size method
  navigation_->ClearDetectedObjects();
  for (int i = 0; i < size; i++) {
    // cout << bb_msg.bounding_boxes[i].Class << "\n";

    // Construct a list of all stop signs / cars / persons, or empty list if none
    if (bb_msg.bounding_boxes[i].Class == "stop sign" || 
        bb_msg.bounding_boxes[i].Class == "car" || bb_msg.bounding_boxes[i].Class == "truck" ||
        bb_msg.bounding_boxes[i].Class == "bus" || bb_msg.bounding_boxes[i].Class == "person") {
      navigation_->AddDetectedObject(bb_msg.bounding_boxes[i]);
    }
    /*
    if ((bb_msg.bounding_boxes[i].Class == "fire hydrant") or
        (bb_msg.bounding_boxes[i].Class == "person")) {
      ROS_INFO("Found fire hydrant or person/people");

      ros::Time time_now = ros::Time::now();
      if (time_now - last_email_sent  >= ros::Duration(10)) { //to prevent sending too many emails - causing to be marked as spam source
        sendMail(email_to.c_str(), email_from.c_str(), "rosbot patrol node","I have found something strange it could be invader");
        last_email_sent = time_now;
      }
    }
    */
  }

  // Call methods
  navigation_->SetStopSignDist();
  navigation_->WaitForCarAtIntersection();
  navigation_->WaitForPedestrian();
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  signal(SIGINT, SignalHandler);
  // Initialize ROS.
  ros::init(argc, argv, "navigation", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  navigation_ = new Navigation(FLAGS_map, &n);

  ros::Subscriber velocity_sub =
      n.subscribe(FLAGS_odom_topic, 1, &OdometryCallback);
  ros::Subscriber localization_sub =
      n.subscribe(FLAGS_loc_topic, 1, &LocalizationCallback);
  ros::Subscriber laser_sub =
      n.subscribe(FLAGS_laser_topic, 1, &LaserCallback);
  ros::Subscriber goto_sub =
      n.subscribe("/move_base_simple/goal", 1, &GoToCallback);
      
  ros::Subscriber darknet_sub =
      n.subscribe("/darknet_ros/bounding_boxes", 1, darknetCallback);

  RateLoop loop(20.0);
  while (run_ && ros::ok()) {
    ros::spinOnce();
    navigation_->Run();
    loop.Sleep();
  }
  delete navigation_;
  return 0;
}
