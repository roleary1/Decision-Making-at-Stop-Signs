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
\file    particle-filter.cc
\brief   Particle Filter Starter Code
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/math/statistics.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"
#include "visualization/visualization.h"
#include "amrl_msgs/VisualizationMsg.h"

using geometry::Line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using vector_map::VectorMap;

DEFINE_double(num_particles, 50, "Number of particles");

namespace particle_filter {

const float kEpsilon = 1e-5;
const float LIDAR_STDEV = .5;
const float GAMMA = .2;

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

void ParticleFilter::GetPredictedPointCloud(const Vector2f& loc,
                                            const float angle,
                                            int num_rays,
                                            int skip,
                                            float range_min,
                                            float range_max,
                                            float angle_min,
                                            float angle_max,
                                            vector<Vector2f>* scan_ptr,
                                            amrl_msgs::VisualizationMsg& vis_msg_) {

  if(!odom_initialized_) {
    return;
  }
                                              
  // Return if we are receiving a scan with no points
  if(angle_min == angle_max && range_max == 0) {
    return;
  }

  angle_min += angle;
  angle_max += angle;

  // Modify loc to laser frame instead of base_link
  Vector2f laserLoc(loc[0] + 0.2 * cos(angle), loc[1] + 0.2 * sin(angle));

  vector<Vector2f>& scan = *scan_ptr;

  scan.clear();
  scan.resize((int)(num_rays / skip));
  
  float angle_increment = (angle_max-angle_min)/(num_rays-1);

  unsigned int index = 0;
  for(double angle = angle_min; angle < angle_max; angle += (angle_increment * skip)) { 
    if (index == scan.size()) break;

    double distance = range_max;
  
    // Calculate endpoint in map frame
    double x_from_laser = distance * cos(angle);
    double y_from_laser = distance * sin(angle);
    Vector2f endpoint(laserLoc[0]+x_from_laser, laserLoc[1]+y_from_laser);

    // Create line for ray
    Line2f current_ray(laserLoc, endpoint);
  
    // Loop through the map to find intersections
    Vector2f intersection_point(endpoint); // Return variable
    CHECK((unsigned int)index < scan.size());
    
    scan[index] = intersection_point;

    for (size_t i = 0; i < map_.lines.size(); ++i) {
      const Line2f map_line = map_.lines[i];

      bool intersects = map_line.Intersection(current_ray, &intersection_point);
      if (intersects) {
        // if we find intersection and it is closer to us than before, we update coordinates
        float curr_intersect_dist_to_car = (intersection_point-loc).norm();
        float old_intersect_dist_to_car = (scan[index]-loc).norm();
        if (curr_intersect_dist_to_car < old_intersect_dist_to_car){
          scan[index] = intersection_point; 
        }
      } else {
        // if no intersection, we keep point cloud particles at range_max distance
      }
    }

    index++;
  }
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            Particle* p_ptr,
                            amrl_msgs::VisualizationMsg& vis_msg_) {
  if(!odom_initialized_) {
    return;
  }
  float d_short = 0.2;
  float d_long = 0.3;
  float s_min = .2;
  float s_max = 5;

  int skip = 10;
  vector<Vector2f> predicted_scan;

  GetPredictedPointCloud(p_ptr->loc, p_ptr->angle, ranges.size(), skip, range_min, range_max, angle_min, angle_max, &predicted_scan, vis_msg_);
  double new_weight = 1;

  // for visualization
  Vector2f laserLoc(p_ptr->loc[0] + 0.2 * cos(p_ptr->angle), p_ptr->loc[1] + 0.2 * sin(p_ptr->angle));

  // loop through the predicted scan, compare with point cloud
  for (unsigned int i = 0; i < predicted_scan.size(); i++) {
    CHECK(i*skip < ranges.size());
    if(ranges[i*skip] != range_max) {
      CHECK_LT(i, predicted_scan.size());
      CHECK_NOTNULL(p_ptr);
      float predicted_dist_to_car = (predicted_scan[i] - p_ptr->loc).norm();
      float actual_dist_to_car = ranges[i * skip];
      float dist_comparison = 0.0;

      if (actual_dist_to_car < s_min || actual_dist_to_car > s_max) {
        // visualization::DrawLine(laserLoc, predicted_scan[i], 0x000000, vis_msg_);
        continue;
      }
      else if (actual_dist_to_car < predicted_dist_to_car - d_short)
      {
        dist_comparison = -pow(d_short, 2) / pow(LIDAR_STDEV, 2);
        // visualization::DrawLine(laserLoc, predicted_scan[i], 0x0000FF, vis_msg_);
      }
      else if (actual_dist_to_car > predicted_dist_to_car + d_long)
      {
        dist_comparison = -pow(d_long, 2) / pow(LIDAR_STDEV, 2);
        // visualization::DrawLine(laserLoc, predicted_scan[i], 0xFF0000, vis_msg_);
      }
      else
      {
        dist_comparison = -pow(predicted_dist_to_car - actual_dist_to_car, 2) / pow(LIDAR_STDEV, 2);
        // visualization::DrawLine(laserLoc, predicted_scan[i], 0x00FF00, vis_msg_);
      }

      new_weight += GAMMA * dist_comparison;
    }
  }
  p_ptr->weight = exp(new_weight);
}

void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.

  vector<Particle> new_particles;
  double sum_weights = 0;

  // Sum the weights of all particles
  for(int i = 0; i < FLAGS_num_particles; i++) {
    sum_weights += particles_[i].weight;
  }

  // Generate new particles
  for(int i = 0; i < FLAGS_num_particles; i++) {
    double sample = rng_.UniformRandom(0, sum_weights);
    int index = 0;
    while(sample >= 0) {
      sample -= particles_[index].weight;
      index++;
    }
    index--;

    Particle replicate;
    replicate.loc = particles_[index].loc;
    replicate.angle = particles_[index].angle;
    // New particles are unweighted
    replicate.weight = 1;

    new_particles.push_back(replicate);
  }

  particles_ = new_particles;
}

int resample_reset = 2;
int update_dist_reset = 0.1;
int update_angle_reset = 0.0872665 * 2;

int resample_count = 0;
float update_dist = 0; 
float update_angle = 0;

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max,
                                  amrl_msgs::VisualizationMsg& vis_msg_) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.
  if(!odom_initialized_) {
    return;
  }
  if(update_dist > update_dist_reset || update_angle > update_angle_reset) {
    Particle* best_p = &particles_[0];
    CHECK(ranges.size() > 0);
    // Call update on every particle to give it a new weight
    for(int i = 0; i < FLAGS_num_particles; i++) {
      Update(ranges, range_min, range_max, angle_min, angle_max, &particles_[i], vis_msg_);
      if(particles_[i].weight > best_p->weight) {
        best_p = &particles_[i];
      }
    }

    for(int i = 0; i < FLAGS_num_particles; i++) {
      particles_[i].weight = particles_[i].weight / best_p->weight;
    }

    // Resample our particles every few updates
    resample_count++;
    if(resample_count >= resample_reset) {
      Resample();
      resample_count = 0;
    }

    update_dist = 0;
    update_angle = 0;
  }

  // visualization::DrawCross(best_p->loc, .01, 0x00FF00, vis_msg_);
}

float k1 = .1;
float k2 = .087;
float k3 = .2;
float k4 = .25;

void ParticleFilter::ObserveOdometry(const Vector2f& odom_loc,
                                     const float odom_angle) {
  if(!odom_initialized_) {
    cout << "odom not initialized\n";
    return;
  }
  
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.

  // Compute the movement change in terms of delta x, delta y, delta theta
  Vector2f locDelta = odom_loc - prev_odom_loc_;
  float deltaTheta = atan2(sin(odom_angle - prev_odom_angle_), 
                           cos(odom_angle - prev_odom_angle_));

  // for choosing when to update
  update_dist += abs(locDelta.norm());
  update_angle += abs(deltaTheta);

  Eigen::Rotation2Df toPrev(-prev_odom_angle_);
  Vector2f prevOdomLoc = toPrev * locDelta;

  // For each particle:
  for(int i = 0; i < FLAGS_num_particles; i++) {
    Eigen::Rotation2Df toMap(particles_[i].angle);
    Vector2f mapLoc = toMap * prevOdomLoc;

    // Compute the std deviations for the motion model
    float stdDevTranslation = k1*sqrt(pow(mapLoc[0],2) + pow(mapLoc[1],2)) + k2*abs(deltaTheta);
    float stdDevRotation = k3*sqrt(pow(mapLoc[0],2) + pow(mapLoc[1],2)) + k4*abs(deltaTheta);

    // Update particle pose by movement change
    // Sample errors ex, ey, ethetha from normal distributions
    float ex = rng_.Gaussian(mapLoc[0], stdDevTranslation);
    float ey = rng_.Gaussian(mapLoc[1], stdDevTranslation);
    float etheta = rng_.Gaussian(deltaTheta, stdDevRotation);

    // Add errors to the particle pose
    particles_[i].loc[0] = particles_[i].loc[0] + ex;
    particles_[i].loc[1] = particles_[i].loc[1] + ey;
    particles_[i].angle  = particles_[i].angle + etheta;
  }

  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  map_.Load(map_file);
  particles_.clear();
  // initialize in a gaussian dist around loc and angle

  float INITIAL_EPSILON = .1;
  for (int i = 0; i < FLAGS_num_particles; i++){
    Particle p;
    p.loc[0] = rng_.Gaussian(loc[0], INITIAL_EPSILON);
    p.loc[1] = rng_.Gaussian(loc[1], INITIAL_EPSILON);
    p.angle = rng_.Gaussian(angle, INITIAL_EPSILON);
    particles_.push_back(p);
  }

  odom_initialized_ = true;
  // Make sure update is called immediately on next laser
  update_dist = 10;
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  if(!odom_initialized_) {
    return;
  }
  if (particles_.size() != FLAGS_num_particles){
    cout << "WRONG NUMBER PARTICLES!\n";
    return;
  } 
  if (!loc_ptr) return;
  if (!angle_ptr) return;

  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  
  Vector2f locTotal(0, 0);
  float sineTotal = 0.0;
  float cosTotal = 0.0;
  float weightTotal = 0;
  for(int i = 0; i < FLAGS_num_particles; i++){
    locTotal += particles_[i].loc * particles_[i].weight;
    sineTotal += sin(particles_[i].angle) * particles_[i].weight;
    cosTotal += cos(particles_[i].angle) * particles_[i].weight;
    weightTotal += particles_[i].weight;
    //cout << "i: " << i << " loc: " << particles_[i].loc[0] 
    //  << " " << particles_[i].loc[1] << " angle: " << particles_[i].angle
    //  << " weight: " << particles_[i].weight << "\n";
  }
  
  loc = locTotal / weightTotal;
  angle = atan2(sineTotal / weightTotal, cosTotal / weightTotal);
}


}  // namespace particle_filter
