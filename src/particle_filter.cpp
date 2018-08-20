/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include "particle_filter.h"
#include <math.h>
#include <algorithm>
#include <functional>
#include <iostream>
#include <iterator>
#include <map>
#include <numeric>
#include <random>
#include <sstream>
#include <string>

#define EPS 0.0001

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  // Creates a normal (Gaussian) distribution for x, y and theta for random noise (from GPS uncertainty).
  normal_distribution<double> dist_x{x, std[0]};
  normal_distribution<double> dist_y{y, std[1]};
  normal_distribution<double> dist_theta{theta, std[2]};

  particles_.resize(num_particles_);
  _weights.resize(num_particles_);

  for (int i = 0; i < num_particles_; ++i) {
    particles_[i].id = i;
    particles_[i].x = dist_x(_rand_gen);
    particles_[i].y = dist_y(_rand_gen);
    particles_[i].theta = dist_theta(_rand_gen);

    // Just for clarity, this will be reinizialized at each updateWeights call
    particles_[i].weight = 1.0;
    _weights[i] = 1.0;
  }

  _is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
  // Creates a normal distribution centered around 0 for x, y and theta for noise that is going to
  // be added later on for each particle
  normal_distribution<double> dist_x{0, std_pos[0]};
  normal_distribution<double> dist_y{0, std_pos[1]};
  normal_distribution<double> dist_theta{0, std_pos[2]};

  for (Particle& particle : particles_) {
    // Predicts the new position
    if (abs(yaw_rate) < EPS) {
      particle.x += velocity * delta_t * cos(particle.theta);
      particle.y += velocity * delta_t * sin(particle.theta);
    } else {
      particle.x += velocity / yaw_rate * (sin(particle.theta + yaw_rate * delta_t) - sin(particle.theta));
      particle.y += velocity / yaw_rate * (cos(particle.theta) - cos(particle.theta + yaw_rate * delta_t));
      particle.theta += yaw_rate * delta_t;
    }
    // Adds noise, note that the std_pos contains the uncertainty of the GPS position so this is not technically right
    // as the noise should be computed in terms of uncertainity of the sensor measurement (e.g. velocity, yaw rate) but
    // it will do for this project.
    particle.x += dist_x(_rand_gen);
    particle.y += dist_y(_rand_gen);
    particle.theta += dist_theta(_rand_gen);
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const std::vector<LandmarkObs>& observations, const Map& map_landmarks) {
  // Terms used later on in the multi-variate gaussian
  const double std_x_2 = 2.0 * std_landmark[0] * std_landmark[0];
  const double std_y_2 = 2.0 * std_landmark[1] * std_landmark[1];
  // Normalization term
  const double gauss_norm = 2.0 * M_PI * std_landmark[0] * std_landmark[1];

  for (int i = 0; i < num_particles_; ++i) {
    vector<int> associations;
    vector<double> sense_x;
    vector<double> sense_y;

    // Temporary vector to hold the landmark that are in the range of the particle (e.g. if the particle is considered
    // the car position)
    vector<Map::single_landmark_s> landmarks_in_range =
        _getLandmarksInRange(sensor_range, particles_[i], map_landmarks);

    particles_[i].weight = 1.0;

    for (const LandmarkObs& observation : observations) {
      // Converts the observation from car's coordinates to map coordinates (using the particle coordinate as reference)
      LandmarkObs converted_obs = _convertToMapCoordinates(particles_[i], observation);

      // Finds the landmark that is closest to the observation
      auto closest_landmark = _findClosestLandmark(converted_obs, landmarks_in_range);

      // Distance between the observation and the closest landmark
      double closest_landmark_x_dist, closest_landmark_y_dist;

      // If no landmark is in range uses a large value to reduce the probability
      if (closest_landmark == landmarks_in_range.end()) {
        closest_landmark_x_dist = sensor_range;
        closest_landmark_y_dist = sensor_range;
      } else {
        closest_landmark_x_dist = converted_obs.x - closest_landmark->x_f;
        closest_landmark_y_dist = converted_obs.y - closest_landmark->y_f;

        associations.push_back(closest_landmark->id_i);
        sense_x.push_back(converted_obs.x);
        sense_y.push_back(converted_obs.y);
      }

      // Updates the weight using the multi-variate gaussian
      double exponent = closest_landmark_x_dist * closest_landmark_x_dist / std_x_2 +
                        closest_landmark_y_dist * closest_landmark_y_dist / std_y_2;

      particles_[i].weight *= exp(-exponent) / gauss_norm;
    }

    _weights[i] = particles_[i].weight;
    setAssociations(particles_[i], associations, sense_x, sense_y);
  }
}

void ParticleFilter::resample() {
  // Resample particles with replacement with probability proportional to their weight.
  discrete_distribution<int> dist_p(_weights.begin(), _weights.end());

  vector<Particle> particles_new(num_particles_);

  for (int i = 0; i < num_particles_; ++i) {
    particles_new[i] = particles_[dist_p(_rand_gen)];
  }

  particles_ = move(particles_new);
}

void ParticleFilter::setAssociations(Particle& particle, const std::vector<int>& associations,
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y) {
  // particle: the particle to assign each listed association, and association's (x,y) world
  // coordinates mapping to
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates

  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}
string ParticleFilter::getSenseX(Particle best) {
  vector<double> v = best.sense_x;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}
string ParticleFilter::getSenseY(Particle best) {
  vector<double> v = best.sense_y;
  stringstream ss;
  copy(v.begin(), v.end(), ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}

inline vector<Map::single_landmark_s> ParticleFilter::_getLandmarksInRange(double sensor_range,
                                                                           const Particle& particle, const Map& map) {
  vector<Map::single_landmark_s> filtered_landmarks;

  // Filters the landmarks that are not in the sensor range (from the position of the particle)
  copy_if(map.landmark_list.begin(), map.landmark_list.end(), back_inserter(filtered_landmarks),
          [sensor_range, &particle](const Map::single_landmark_s& landmark) {
            return dist(landmark.x_f, landmark.y_f, particle.x, particle.y) <= sensor_range;
          });

  return filtered_landmarks;
}

inline LandmarkObs ParticleFilter::_convertToMapCoordinates(const Particle& particle, const LandmarkObs& observation) {
  double x_map = particle.x + cos(particle.theta) * observation.x - sin(particle.theta) * observation.y;
  double y_map = particle.y + sin(particle.theta) * observation.x + cos(particle.theta) * observation.y;

  return LandmarkObs{observation.id, x_map, y_map};
}

inline std::vector<Map::single_landmark_s>::const_iterator ParticleFilter::_findClosestLandmark(
    const LandmarkObs& observation, const vector<Map::single_landmark_s>& landmarks) {
  double min_distance = numeric_limits<double>::max();
  auto closest_landmark = landmarks.end();

  for (auto it = landmarks.begin(); it != landmarks.end(); ++it) {
    double distance = dist(observation.x, observation.y, it->x_f, it->y_f);
    if (distance < min_distance) {
      min_distance = distance;
      closest_landmark = it;
    }
  }

  return closest_landmark;
}
