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
\file    navigation.h
\brief   Interface for reference Navigation class.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <vector>

#include "eigen3/Eigen/Dense"

#include "vector_map/vector_map.h"
#include <darknet_ros_msgs/BoundingBoxes.h>

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
  class NodeHandle;
}  // namespace ros

namespace navigation {

struct PathOption {
  float curvature;
  float clearance;
  float free_path_length;
  Eigen::Vector2f obstruction;
  Eigen::Vector2f closest_point;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

class Navigation {
 public:

   // Constructor
  explicit Navigation(const std::string& map_file, ros::NodeHandle* n);

  // Helper method to generate a graph for the current map
  void GenerateGraph();

  // Used in callback from localization to update position.
  void UpdateLocation(const Eigen::Vector2f& loc, float angle);

  // Used in callback for odometry messages to update based on odometry.
  void UpdateOdometry(const Eigen::Vector2f& loc,
                      float angle,
                      const Eigen::Vector2f& vel,
                      float ang_vel);

  // Updates based on an observed laser scan
  void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
                         double time);

  // Main function called continously from main
  void Run();

  // Used to set the next target pose.
  void SetNavGoal(const Eigen::Vector2f& loc, float angle);
  

  // returns the system latency with assumption of .15 seconds
  float GetLatency();

  // returns clearance
  float getClearance(const std::vector<Eigen::Vector2f>& cloud, float curvature, float freePathLength);

  // returns distance to goal
  float DistanceToGoal(float curvature, float freePathLength, Eigen::Vector2f intermediateGoal);

  // 1 Dimensional Time Control Optimal Problem Solver
  // Returns new velocity
  float OneDim(float displacementRemaining, float velocity);

  // Detects Obstacles based on Point Cloud and Curvature
  // Returns the Free Path Length
  float ObstacleDetection(const std::vector<Eigen::Vector2f>& cloud, 
                          float curvature);

  // Find our intermediate goal from the current path
  Eigen::Vector2f GetIntermediateGoal(float goal_dist);

  // Find the maximum we would travel along this arc
  float MaxTravelToIntermediateGoal(float curvature, Eigen::Vector2f intermediateGoal);

  void VisualizePathing();

  float GetObjectDepth(darknet_ros_msgs::BoundingBox box);

  void SetStopSignDist();

  void WaitForCarAtIntersection();

  void WaitForPedestrian();

  void AddDetectedObject(darknet_ros_msgs::BoundingBox box);

  void ClearDetectedObjects();

 private:

  // Whether odometry has been initialized.
  bool odom_initialized_;
  // Whether localization has been initialized.
  bool localization_initialized_;
  // Current robot location.
  Eigen::Vector2f robot_loc_;
  // Current robot orientation.
  float robot_angle_;
  // Current robot velocity.
  Eigen::Vector2f robot_vel_;
  // Current robot angular speed.
  float robot_omega_;
  // Odometry-reported robot location.
  Eigen::Vector2f odom_loc_;
  // Odometry-reported robot angle.
  float odom_angle_;
  // Odometry-reported robot starting location.
  Eigen::Vector2f odom_start_loc_;
  // Odometry-reported robot starting angle.
  float odom_start_angle_;
  // Latest observed point cloud.
  std::vector<Eigen::Vector2f> point_cloud_;
  // Whether navigation is complete.
  bool nav_complete_;

  // whether a stop sign has been detected in front of us
  bool stop_detected_ = false;
  bool post_stop_ = false;

  bool just_woke_ = false;
  bool recent_check_ = false;

  ros::Time beginIgnoreObjects; // used to drive for 3 seconds while ignoring detected objects
  ros::Time waitForCarDetect; // used to drive for 3 seconds while ignoring detected objects
  bool wait_detect_car = false;

  // hard coded stop sign location for now
  float stop_sign_dist_ = 0;
  // distance to stop before a stop sign
  float stop_before_dist_ = 0.75f;

  // whether a car has been detected in front of us
  bool car_detected_ = false;
  bool car_at_intersection_before_ = false;
  // detected car distance
  float other_car_distance_ = 0;

  bool ped_detected_ = false;
  bool ped_at_intersection_ = false;
  float object_detect_bound_ = 5;   // distance for which we care about detected objects
                                      // should be about the width of one intersection

  std::vector<darknet_ros_msgs::BoundingBox> detected_objects_;

  // Navigation goal location.
  Eigen::Vector2f nav_goal_loc_;
  // Navigation goal angle.
  float nav_goal_angle_;
  // Map of the environment.
  vector_map::VectorMap map_;
};

}  // namespace navigation

#endif  // NAVIGATION_H
