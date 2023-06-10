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
\file    navigation.cc
\brief   Starter code for navigation.
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "ros/package.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include <cmath>
#include <map>

#include "simple_queue.h"
#include "simple_graph.h"

#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud2_iterator.h"


using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using Eigen::Vector2f;
using geometry::Line2f;
using std::string;
using std::vector;

using namespace math_util;
using namespace ros_helpers;

DEFINE_double(cp1_distance, 2.5, "Distance to travel for 1D TOC (cp1)");
DEFINE_double(cp1_curvature, 0.5, "Curvature for arc path (cp1)");

DEFINE_double(cp2_curvature, 0.5, "Curvature for arc path (cp2)");

namespace
{
  ros::Publisher drive_pub_;
  ros::Publisher viz_pub_;
  VisualizationMsg local_viz_msg_;
  VisualizationMsg global_viz_msg_;
  AckermannCurvatureDriveMsg drive_msg_;
  // Epsilon value for handling limited numerical precision.
  const float kEpsilon = 1e-5;
  const bool PLANNING_TEST_NO_NAVIGATION = false;
  bool path_complete_ = true;
} // namespace

namespace navigation
{
  string GetMapFileFromName(const string &map)
  {
    string maps_dir_ = ros::package::getPath("amrl_maps");
    return maps_dir_ + "/" + map + "/" + map + ".vectormap.txt";
  }

  Navigation::Navigation(const string &map_name, ros::NodeHandle *n) : odom_initialized_(false),
                                                                       localization_initialized_(false),
                                                                       robot_loc_(0, 0),
                                                                       robot_angle_(0),
                                                                       robot_vel_(0, 0),
                                                                       robot_omega_(0),
                                                                       nav_complete_(true),
                                                                       nav_goal_loc_(0, 0),
                                                                       nav_goal_angle_(0)
  {
    map_.Load(GetMapFileFromName(map_name));
    drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
        "ackermann_curvature_drive", 1);
    viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
    local_viz_msg_ = visualization::NewVisualizationMessage(
        "base_link", "navigation_local");
    global_viz_msg_ = visualization::NewVisualizationMessage(
        "map", "navigation_global");
    InitRosHeader("base_link", &drive_msg_.header);

    GenerateGraph();

    cout << "Finished initialization\n";
  }

  float map_start = -50;
  float map_end = 50;
  float node_dist = .25;

  Graph graph_;

  void Navigation::GenerateGraph() 
  {
    // Initialize entire grid at once (Option 2)
    Graph graph(node_dist, map_start, map_start, map_end, map_end);
    
    int count = 0;
    for(float x = map_start; x < map_end; x += node_dist) {
      for(float y = map_start; y < map_end; y += node_dist) {
        Node new_node;
        new_node.pos = Vector2f(x, y);

        // Check below
        int adj = graph.getDirectionAdjust(4);
        if(count+adj >= 0 && count % graph.column_height != 0) {
          new_node.edges[4] = count+adj;
          graph.nodes[count+adj].edges[0] = count;
        }
        // Check left-down
        adj = graph.getDirectionAdjust(5);
        if(count+adj >= 0 && count % graph.column_height != 0) {
          new_node.edges[5] = count+adj;
          graph.nodes[count+adj].edges[1] = count;
        }
        // Check straight-left
        adj = graph.getDirectionAdjust(6);
        if(count+adj >= 0) {
          new_node.edges[6] = count+adj;
          graph.nodes[count+adj].edges[2] = count;
        }
        // Check left-up
        adj = graph.getDirectionAdjust(7);
        if(count+adj >= 0) {
          new_node.edges[7] = count+adj;
          graph.nodes[count+adj].edges[3] = count;
        }

        graph.nodes.insert(pair<int, Node>(count, new_node));

        count++;
      }
    }

    // Remove edges on the map where map lines intersect
    for (unsigned int i = 0; i < map_.lines.size(); ++i) {
      const Line2f map_line = map_.lines[i];

      // Get bottom-left and top-right nodes of bounding box around the map line
      Vector2f p0 = map_line.p0;
      Vector2f p1 = map_line.p1;

      float top_x = p0[0] > p1[0] ? p0[0] : p1[0];
      float bot_x = p0[0] < p1[0] ? p0[0] : p1[0];
      float top_y = p0[1] > p1[1] ? p0[1] : p1[1];
      float bot_y = p0[1] < p1[1] ? p0[1] : p1[1];

      float round_value = 1/node_dist;
      Vector2f top_right(ceil(top_x*round_value)/round_value, ceil(top_y*round_value)/round_value);
      Vector2f bot_left(floor(bot_x*round_value)/round_value, floor(bot_y*round_value)/round_value);
      
      // Iterate through all nodes in bounding box
      for(float x = bot_left[0]; x <= top_right[0]; x+= node_dist) {
        for(float y = bot_left[1]; y <= top_right[1]; y+= node_dist) {
          Vector2f curr_pos(x, y);
          int curr = graph.getNodeNumber(Vector2f(x, y));

          // Iterate through every edge
          for(int j = 0; j < 8; j++) {
            int node_at_edge = graph.nodes[curr].edges[j];

            if(node_at_edge == -1) {
              continue;
            } 

            // Remove the edge both ways
            graph.nodes[curr].edges[j] = -1;
            int opposite = (j + 4) % 8;
            graph.nodes[node_at_edge].edges[opposite] = -1;
          }
        }
      }
    }

    graph_ = graph; 
  }

  vector<Line2f> map_path_;

  void Navigation::SetNavGoal(const Vector2f &loc, float angle)
  {
    cout << "NavGoal Received:\n" << loc << "\n" << angle << "\n";
    nav_goal_loc_ = loc;
    nav_goal_angle_ = angle;
    
    // Assign start and end points to nodes
    int start_node = graph_.getNodeNumber(robot_loc_);
    int end_node = graph_.getNodeNumber(nav_goal_loc_);

    SimpleQueue<int, float> frontier;
    std::map <int, int> parent; // Key: Node, Value: its predecessor
    std::map <int, float> cost;

    frontier.Push(start_node, 0);
    parent.insert(pair<int, int>(start_node, -1));
    cost.insert(pair<int, float>(start_node, 0));

    while(!frontier.Empty()) {
      int current = frontier.Pop();
      if(current == end_node) {
        break;
      }
      // Loop through neighbors
      for(int i = 0; i < 8; i++) {
        int next_node = graph_.nodes[current].edges[i];
        if(next_node != -1) {
          float new_cost = cost[current] + graph_.getCost(i);
          if(cost.count(next_node) == 0 || new_cost < cost[next_node]) {
            // Insertion or edit
            // Heuristic = euclidean distance to goal
            float heuristic = (graph_.nodes[next_node].pos - nav_goal_loc_).norm();
            frontier.Push(next_node, new_cost + heuristic);
            cost[next_node] = new_cost; 
            parent[next_node] = current;
          }
        }
      }
    }

    map_path_.clear();

    // Convert path to lines
    int curr_node = end_node;
    while(parent.size() > 0 && parent[curr_node] != -1) {
      int parent_node = parent[curr_node];
      Line2f curr_line(graph_.nodes[curr_node].pos, graph_.nodes[parent_node].pos);
      map_path_.push_back(curr_line);

      curr_node = parent_node;
    }

    path_complete_ = false;

    cout << "Finished setting nav goal\n";
  }

  void Navigation::VisualizePathing() {
    // Visualize end node point
    int end_node = graph_.getNodeNumber(nav_goal_loc_);
    visualization::DrawCross(graph_.nodes[end_node].pos, .1, 0x00FF00, global_viz_msg_);

    // Visualize path
    for(unsigned int i = 0; i < map_path_.size(); i++) {
      visualization::DrawLine(map_path_[i].p0, map_path_[i].p1, 0x0000FF, global_viz_msg_);
    }
  }

  void Navigation::UpdateLocation(const Eigen::Vector2f &loc, float angle)
  {
    localization_initialized_ = true;
    robot_loc_ = loc;
    robot_angle_ = angle;
  }

  // Odometry
  Vector2f odom_loc_last(0.0, 0.0);
  float odom_angle_last;

  void Navigation::UpdateOdometry(const Vector2f &loc,
                                  float angle,
                                  const Vector2f &vel,
                                  float ang_vel)
  {
    if(PLANNING_TEST_NO_NAVIGATION) {
      return;
    }

    robot_omega_ = ang_vel;
    robot_vel_ = vel;
    if (!odom_initialized_)
    {
      odom_start_angle_ = angle;
      odom_start_loc_ = loc;
      odom_initialized_ = true;
      odom_loc_ = loc;
      odom_angle_ = angle;
      return;
    }
    odom_loc_ = loc;
    odom_angle_ = angle;

    // Update last odom angle and positions
    odom_loc_last = odom_loc_;
    odom_angle_last = odom_angle_;
  }

  void Navigation::ObservePointCloud(const vector<Vector2f> &cloud,
                                     double time)
  {
    point_cloud_ = cloud;
  }

  const double CAR_WIDTH = 0.281;
  const double CAR_LENGTH = 0.535;
  const double OBSTACLE_MARGIN = 0.1;
  const double MAX_LASER_RANGE = 10;
  // This figure is an ESTIMATION given by CP2. Must MEASURE ON THE CAR
  const double CAR_BACK_TO_BASE = 0.135;

  // Width from origin to margin
  const float w = CAR_WIDTH / 2 + OBSTACLE_MARGIN;
  // Length from origin to margin
  const float h = CAR_LENGTH - CAR_BACK_TO_BASE + OBSTACLE_MARGIN;

  const double MAX_VELOCITY = 0.3;
  const double MAX_ACCEL = 3.0;
  const double TIME_STEP = 0.05;
  bool flags_read = false;
  double timePassed = 0; // For testing
  double v1 = 0, v2 = 0, v3 = 0;
  const float C_MAX = CAR_WIDTH; // clearance on one side (i.e. to the left, right of car obstacle buffer)

  void Navigation::Run()
  {
    // This function gets called 20 times a second to form the control loop.

    // Clear previous visualizations.
    visualization::ClearVisualizationMsg(local_viz_msg_);
    visualization::ClearVisualizationMsg(global_viz_msg_);

    // If odometry has not been initialized, we can't do anything.
    if (!odom_initialized_)
    {
      return;
    }

    if (!flags_read)
    {
      //curvature = FLAGS_cp2_curvature;
      flags_read = true;
    }

    float intermediateGoalDistance = 1;
    Vector2f intermediateGoal(0, 0);

    if(path_complete_) {
      drive_msg_.velocity = 0;
    } 
    else {
      intermediateGoal = GetIntermediateGoal(intermediateGoalDistance);

      // Visualize intermediate goal
      visualization::DrawCross(intermediateGoal, .1, 0xFFA500, local_viz_msg_);

      // Visualize Stop Sign
      // visualization::DrawCross(stop_sign_loc_, .1, 0xFF0000, global_viz_msg_);

      // Check if path is complete
      if((robot_loc_ - nav_goal_loc_).norm() < 0.1) {
        path_complete_ = true;
      }

      int NUM_PATHS = 21;
      float MAX_TURNING_ANGLE = 2;
      float w1 = 5; // weight for clearance, old: 0.05
      float w2 = -.1; // weight for distance to goal, old: -.1
      float bestCurvature = 0;
      float bestFreePathLength = 0;
      float maxHeuristic = __FLT_MIN__;
      int i;
      float curvature;
      float curvatureUpdate = 2*MAX_TURNING_ANGLE / (NUM_PATHS - 1);
      for (i = 0, curvature = -1 * MAX_TURNING_ANGLE; i < NUM_PATHS; i++, curvature += curvatureUpdate)
      {

        float freePathLength = std::min(ObstacleDetection(point_cloud_, curvature), 
                                  MaxTravelToIntermediateGoal(curvature, intermediateGoal));
        float clearance = getClearance(point_cloud_, curvature, freePathLength);
        float distToGoal = DistanceToGoal(curvature, freePathLength, intermediateGoal);
        float heuristic = freePathLength + w2 * distToGoal + w1 * clearance;

        // Small bias for straight movement
        if(abs(curvature) < kEpsilon) {
          heuristic *= 1.2;
        }

        if (heuristic > maxHeuristic)
        {
          maxHeuristic = heuristic;
          bestCurvature = curvature;
          bestFreePathLength = freePathLength;
        }

      }
      drive_msg_.curvature = bestCurvature;

      // calculate latency and subtract from freePathLength
      // float latencyDistance = GetLatency();
      // drive_msg_.velocity = OneDim(freePathLength - latencyDistance, robot_vel_[0]);
      
      // check if we can end 'just exited stop sign' mode
      if(post_stop_) {
        ros::Duration time_passed = ros::Time::now() - beginIgnoreObjects;
        if(time_passed.toSec() >= 3) {
          post_stop_ = false;
        }
      }

      if(just_woke_) {
        // Wait until we do another pedestrian check
        if(!recent_check_) {
          drive_msg_.velocity = 0;
        } else {
          // wait for pedestrians to leave if we see them
          if(ped_detected_) {
            ped_at_intersection_ = true;
            drive_msg_.velocity = 0;
          } else if(car_at_intersection_before_) {
            // check if a car is detected at intersection - wait until it leaves
            drive_msg_.velocity = 0;
          } else {
            post_stop_ = true;
            beginIgnoreObjects = ros::Time::now();
          }

          stop_detected_ = false;
          just_woke_ = false;
        }
      } else if(car_at_intersection_before_ || ped_at_intersection_) {
        // we've stopped at a stop sign and there is a car or ped there - now we wait until it leaves
        // can't just while loop since callback isn't asynchronous
        drive_msg_.velocity = 0;
        if(!car_detected_) {
          // now can continue
          car_at_intersection_before_ = false;
        }
        if(!ped_detected_) {
          ped_at_intersection_ = false;
        }
        if(!car_detected_ && !ped_at_intersection_) {
          post_stop_ = true;
          beginIgnoreObjects = ros::Time::now();
        }
      } else if(stop_detected_ && stop_sign_dist_ <= stop_before_dist_) {
        if(car_detected_) {
          car_at_intersection_before_ = true;
        }
        
        ros::Duration(3).sleep();
        just_woke_ = true;
        recent_check_ = false;
        drive_msg_.velocity = 0;
      } else if(stop_detected_) {
        drive_msg_.velocity = OneDim(std::min(bestFreePathLength, stop_sign_dist_), robot_vel_[0]);
      } else {
        drive_msg_.velocity = OneDim(bestFreePathLength, robot_vel_[0]);
      }
      if(drive_msg_.velocity < 0) drive_msg_.velocity = 0;
    }
    

    v1 = v2;
    v2 = v3;
    v3 = drive_msg_.velocity;

    timePassed += TIME_STEP;

    VisualizePathing();

    // Add timestamps to all messages.
    local_viz_msg_.header.stamp = ros::Time::now();
    global_viz_msg_.header.stamp = ros::Time::now();
    drive_msg_.header.stamp = ros::Time::now();
    // Publish messages.
    viz_pub_.publish(local_viz_msg_);
    viz_pub_.publish(global_viz_msg_);
    drive_pub_.publish(drive_msg_);
  }

  float Navigation::GetLatency()
  {
    return (v1 * TIME_STEP) + (v2 * TIME_STEP) + (v3 * TIME_STEP);
  }

  // Previous goal in global frame
  Vector2f prev_goal(0, 0);

  // Return intermediate goal in local frame
  Vector2f Navigation::GetIntermediateGoal(float goal_dist) {
    // Iterate through the path, starting at its end
    const Vector2f circle_center(robot_loc_[0], robot_loc_[1]);

    Vector2f map_frame_goal(0, 0);
    bool found_intermediate = false;

    for(unsigned int i = 0; i < map_path_.size(); i++) {
      Vector2f intersection1;
      Vector2f insersection2;

      const Vector2f line_start(map_path_[i].p0);
      const Vector2f line_end(map_path_[i].p1);

      int num_intersects = geometry::CircleLineIntersection
        (circle_center, goal_dist, line_start, line_end, &intersection1, &insersection2);

      // Should be impossible with circle and line size (very short)
      if(num_intersects == 2) {
        cout << "ERROR: TOO MANY INTERSECTIONS\n";
      } 
      // Stop at the first intersection found: furthest on the path
      else if (num_intersects == 1) {
        // Remove all lines behind you on the path 
        // This will stop the loop by reducing map_path.size()
        map_path_.erase(map_path_.begin()+(i+1), map_path_.end());
        
        map_frame_goal = intersection1;
        prev_goal = map_frame_goal;
        found_intermediate = true;
      } 
    }

    // Return the last intermediate goal we have found
    if(!found_intermediate) {
      map_frame_goal = prev_goal;
    }

    // Return in local frame
    Vector2f map_diff(map_frame_goal - robot_loc_);
    Eigen::Rotation2Df to_local(-robot_angle_);
    Vector2f local_goal = to_local * map_diff;
    return local_goal;
  }

  float Navigation::ObstacleDetection(const vector<Vector2f> &cloud, float curvature)
  {
    float max_path = MAX_LASER_RANGE;

    // Special case for travelling straight
    if (abs(curvature) < kEpsilon)
    {
      for (Vector2f p : cloud)
      {
        // Check if point is an obstacle
        // NOTE: last term makes sure the point is in front of our car
        if (abs(p[1]) <= w && p[0] > 0)
        {
          // Determine new free path length using equations from lecture
          float obstaclePathLength = p[0] - h;
          if (obstaclePathLength < max_path)
          {
            max_path = obstaclePathLength;
          }
        }
      }
    }
    else
    {
      // Radius of turning
      float r = 1 / curvature;
      float radius = abs(1 / curvature);
      // r1 and r2 in the lecture
      float inner_boundary = radius - w;
      float outer_boundary = sqrt(pow(radius + w, 2) + pow(h, 2));
      // c in the lecture
      Vector2f adjustment(0, r);

      for (Vector2f p : cloud)
      {
        float distance = (p - adjustment).norm();
        // Check if point is an obstacle
        // NOTE: last term makes sure the point is in front of our car
        if (distance >= inner_boundary && distance <= outer_boundary && p[0] > 0)
        {
          
          // Determine new free path length using equations from lecture
          // NOTE: phi = theta - omega

          float phi = atan2(p[0], r - p[1]) - atan2(h, r - w);
          float obstaclePathLength = r * phi;

          if (obstaclePathLength >= 0 && obstaclePathLength < max_path)
          {
            max_path = obstaclePathLength;
          }
        }
      }
    }

    return max_path;
  }
  
  // Does not handle well when intermediate goal is behind us
  float Navigation::MaxTravelToIntermediateGoal(float curvature, Vector2f intermediateGoal) {
    if (abs(curvature) < kEpsilon) {
      return intermediateGoal[0];
    }
    
    float r = 1 / curvature;
    float radius = abs(1 / curvature);

    const Vector2f center_of_circle(0, r);
    const Vector2f intermediate(intermediateGoal);

    // find unit vector for slope between center and intermediate
    float slope = (intermediate[1] - center_of_circle[1]) / (intermediate[0] - center_of_circle[0]);
    Vector2f slope_vector(1, slope);
    Vector2f unit_vector = slope_vector / slope_vector.norm();

    // use unit_vector to get our colinear point p
    Vector2f p = center_of_circle + (unit_vector * radius);    
    
    float d = p.norm();
    float thetaToP = acos(1 - pow(d, 2)/(2 * pow(radius, 2)));
    float maxTravel = thetaToP * radius;

    return maxTravel;
  }

  float Navigation::getClearance(const vector<Vector2f> &cloud, float curvature, float freePathLength)
  {
    // clearance: defined by closest point from on the iside, closest point on the outside, on free path length
    float cInside = C_MAX + OBSTACLE_MARGIN + CAR_WIDTH / 2;
    float cOutside = C_MAX + OBSTACLE_MARGIN + CAR_WIDTH / 2;
    // straight lines
    if (abs(curvature) < kEpsilon)
    {
      for (Vector2f p : cloud)
      {
        if (p[0] > -1 * CAR_BACK_TO_BASE && p[0] < freePathLength + CAR_LENGTH - CAR_BACK_TO_BASE)
        {
          if (p[1] < 0 && p[1] > cInside)
            cOutside = p[1];
          else if (p[1] > 0 && p[1] < cOutside)
            cInside = p[1];
        }
      }
      return cOutside + cInside;
    }
    // arcs
    else
    {
      // Radius of turning
      float r = abs(1 / curvature);
      // r1 and r2 in the lecture
      float inner_boundary = r - w;
      float outer_boundary = sqrt(pow(r + w, 2) + pow(h, 2));
      // c in the lecture
      Vector2f adjustment(0, r);
      Vector2f backCorner(-1 * CAR_BACK_TO_BASE, CAR_WIDTH / 2);
      double thetaMin = acos((adjustment[0] * backCorner[0] + adjustment[1] * backCorner[1]) / (adjustment.norm() * backCorner.norm()));
      double phi = freePathLength / r;
      Vector2f e(r * sin(phi), r * (1 - cos(phi)));
      float omega = atan2(h, r - w);
      float thetaMax = omega + phi;
      for (Vector2f pOriginal : cloud)
      {
        // If we are turning with negative curvature, we will flip all of our points
        // (We do the same above by considering only the absolute value of r)
        Vector2f p(pOriginal[0], -pOriginal[1]);
        float thetaPoint = atan2(p[0], r - p[1]);
        if (thetaPoint > thetaMin && thetaPoint < thetaMax)
        {

          float distToPoint = (p - adjustment).norm();
          // outside case
          if (distToPoint > r && distToPoint - outer_boundary < cOutside)
          {
            cOutside = distToPoint;
          }
          else if (distToPoint < r && inner_boundary - distToPoint < cInside)
          {
            cInside = distToPoint;
          }
        }
      }
      return cOutside + cInside;
    }
  }

  float Navigation::DistanceToGoal(float curvature, float freePathLength, Vector2f intermediateGoal)
  {
    if (abs(curvature) < kEpsilon) {
      // Result of going straight
      Vector2f end_of_straight(freePathLength, 0);
      return (intermediateGoal - end_of_straight).norm();
    }

    float radius = abs(1 / curvature);
    
    double phi = freePathLength / radius;
    Vector2f e(radius * sin(phi), radius * (1-cos(phi)));

    return (intermediateGoal - e).norm();
  }

  float Navigation::OneDim(float displacementRemaining, float velocity)
  {
    float new_velocity = 0;
    // If we overshoot by any amount, decelerate at maximal speed
    if (displacementRemaining < 0)
    {
      new_velocity = velocity - MAX_ACCEL * TIME_STEP;
      if (new_velocity < 0)
      {
        new_velocity = 0;
      }
    }
    // case where we are initially cruising
    else if (velocity == MAX_VELOCITY)
    {
      if (velocity * TIME_STEP + pow(velocity, 2) / (2 * MAX_ACCEL) > displacementRemaining)
      {
        // find the best possible rate of deceleration and set the next velocity
        double deceleration = pow(velocity, 2) / (2 * displacementRemaining);
        new_velocity = velocity - deceleration * TIME_STEP;
      }
      // cruise at max velocity
      else
      {
        new_velocity = velocity;
      }
    }
    // case where we are less than max velocity
    else if (velocity < MAX_VELOCITY)
    {
      double accel_velocity = velocity + MAX_ACCEL * TIME_STEP;
      if (accel_velocity > MAX_VELOCITY)
      {
        accel_velocity = MAX_VELOCITY;
      }

      if ((pow(accel_velocity, 2) / (2 * MAX_ACCEL) - pow(velocity, 2) / (2 * MAX_ACCEL)) + pow(velocity, 2) / (2 * MAX_ACCEL) <= displacementRemaining)
      {
        // accelerate
        new_velocity = accel_velocity;
      }
      else
      {
        // prevent overshooting by slowing down at best possible rate
        double deceleration = pow(velocity, 2) / (2 * displacementRemaining);
        new_velocity = velocity - deceleration * TIME_STEP;
      }
    }
    else
    {
      new_velocity = MAX_VELOCITY;
    }

    return new_velocity;
  }

// static int DEPTH_POINT_SIZE = 16;
static int DEPTH_ROW_WIDTH = 640;
// static int DEPTH_HEIGHT = 480;

  float Navigation::GetObjectDepth(darknet_ros_msgs::BoundingBox box) {
    Eigen::Vector2i center_point((int)(box.xmin + box.xmax)/2, (int)(box.ymin + box.ymax)/2);

    // find the depth of the center point 
    sensor_msgs::PointCloud2 pc;
    pc = *(ros::topic::waitForMessage<sensor_msgs::PointCloud2>
      ("/camera/depth/points", ros::Duration(10)));

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(pc, "x");

    int center_number = center_point.y() * DEPTH_ROW_WIDTH + center_point.x();
    iter_x += center_number;
    //cout << "x: " << iter_x[0] << " y: " << iter_x[1] << " z: " << iter_x[2] << endl;

    return iter_x[2];
  }

  void Navigation::SetStopSignDist() {
    float z_dist = std::numeric_limits<float>::max();;
    stop_detected_ = false;

     for(darknet_ros_msgs::BoundingBox box : detected_objects_) {
      if(!post_stop_ && (box.Class == "stop sign")) {
        stop_detected_ = true;
        z_dist = std::min(z_dist, GetObjectDepth(box));
        cout << "Stop Sign Depth: " << z_dist << endl;
        
        if(!std::isnan(z_dist)) {
          stop_sign_dist_ = z_dist;
        }
      }
     }
  }

  void Navigation::WaitForCarAtIntersection() {
    // depth code same as stop sign check
    float z_dist = std::numeric_limits<float>::max();
    bool prev_detected = car_detected_, just_stopped_waiting = false;

    if(wait_detect_car && ((ros::Time::now() - waitForCarDetect).toSec() > 1)) {
      wait_detect_car = false;
      just_stopped_waiting = true;
    }

    if(!wait_detect_car) 
      car_detected_ = false;

     for(darknet_ros_msgs::BoundingBox box : detected_objects_) {
      if(!post_stop_ && (box.Class == "car" || box.Class == "truck" || box.Class == "bus")) {
        z_dist = std::min(z_dist, GetObjectDepth(box));
        cout << "Car Depth: " << z_dist << endl;

        if(std::isnan(z_dist) || z_dist <= object_detect_bound_) {
          car_detected_ = true;
        }
      }
     }

     // begin waiting for car to reappear
     if(!just_stopped_waiting && prev_detected && !car_detected_) {
       waitForCarDetect = ros::Time::now();
       wait_detect_car = true;
       car_detected_ = true;
     }
  }

  void Navigation::WaitForPedestrian() {
    float z_dist = std::numeric_limits<float>::max();
    ped_detected_ = false;
    recent_check_ = true;

    // depth code same as stop sign check
    for(darknet_ros_msgs::BoundingBox box : detected_objects_) {
      if(!post_stop_ && box.Class == "person") {
        z_dist = std::min(z_dist, GetObjectDepth(box));
        cout << "Pedestrian Depth: " << z_dist << endl;

        if(std::isnan(z_dist) || z_dist <= object_detect_bound_) {
          ped_detected_ = true;
        }
      }
     }
  }

  void Navigation::AddDetectedObject(darknet_ros_msgs::BoundingBox box) {
    detected_objects_.push_back(box);
  }

  void Navigation::ClearDetectedObjects() {
    detected_objects_.clear();
  }

  // One method, go through the list
  // Set car_detected_ etc. to false if we don't find it

} // namespace navigation
