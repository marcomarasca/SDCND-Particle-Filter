/*
 * particle_filter.h
 *
 * 2D particle filter class.
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_

#include <random>
#include "helper_functions.h"

struct Particle {
  int id;
  double x;
  double y;
  double theta;
  double weight;
  std::vector<int> associations;
  std::vector<double> sense_x;
  std::vector<double> sense_y;
};

class ParticleFilter {
 public:
  // Default random engine
  std::mt19937 rand_gen_;

  // Number of particles to draw
  int num_particles_;

  // Flag, if filter is initialized
  bool is_initialized_;

  // Vector of weights of all particles
  std::vector<double> weights_;

  // Set of current particles
  std::vector<Particle> particles_;

  // Constructor
  ParticleFilter() : rand_gen_(std::random_device()()), num_particles_(100), is_initialized_(false) {}

  // @param num_particles Number of particles
  ParticleFilter(int num_particles) : rand_gen_(std::random_device()()), num_particles_(num_particles), is_initialized_(false) {}

  // Destructor
  ~ParticleFilter() {}

  /**
   * init Initializes particle filter by initializing particles to Gaussian
   *   distribution around first position and all the weights to 1.
   * @param x Initial x position [m] (simulated estimate from GPS)
   * @param y Initial y position [m]
   * @param theta Initial orientation [rad]
   * @param std[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
   *   standard deviation of yaw [rad]]
   */
  void init(double x, double y, double theta, double std[]);

  /**
   * prediction Predicts the state for the next time step
   *   using the process model.
   * @param delta_t Time between time step t and t+1 in measurements [s]
   * @param std_pos[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
   *   standard deviation of yaw [rad]]
   * @param velocity Velocity of car from t to t+1 [m/s]
   * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
   */
  void prediction(double delta_t, double std_pos[], double velocity, double yaw_rate);

  /**
   * updateWeights Updates the weights for each particle based on the likelihood of the
   *   observed measurements.
   * @param sensor_range Range [m] of sensor
   * @param std_landmark[] Array of dimension 2 [Landmark measurement uncertainty [x [m], y [m]]]
   * @param observations Vector of landmark observations
   * @param map Map class containing map landmarks
   */
  void updateWeights(double sensor_range, double std_landmark[], const std::vector<LandmarkObs>& observations,
                     const Map& map_landmarks);

  /**
   * resample Resamples from the updated set of particles to form
   *   the new set of particles.
   */
  void resample();

  /*
   * Set a particles list of associations, along with the associations calculated world x,y
   * coordinates This can be a very useful debugging tool to make sure transformations are correct
   * and assocations correctly connected
   */
  void setAssociations(Particle& particle, const std::vector<int>& associations, const std::vector<double>& sense_x,
                       const std::vector<double>& sense_y);

  std::string getAssociations(Particle best);
  std::string getSenseX(Particle best);
  std::string getSenseY(Particle best);

  /**
   * initialized Returns whether particle filter is initialized yet or not.
   */
  const bool initialized() const { return is_initialized_; }

 private:
  /**
   * Returns the list of landmarks in the map that in the range of the given particle.
   *
   * @param sensor_range Range [m] of sensor
   * @param particle The particle
   * @param map Map containing all the landmarks
   * 
   * @return A list of landmaks that are in range of the given particle
   */
  std::vector<Map::single_landmark_s> _getLandmarksInRange(double sensor_range, const Particle& particle, const Map& map);

  /**
   * Converts the given observation to map coordinates
   * 
   * @param particle Reference particle (map coordinates)
   * @param observation The original observation in the car's coordinate
   * 
   * @return A copy of the given observation converted to map coordinates
   */
  LandmarkObs _convertToMapCoordinates(const Particle& particle, const LandmarkObs& observation);

  /**
   * Finds the landmark that is the closest to the given observation (in map coordinates)
   * 
   * @param observation The observation converted to map coordinates
   * 
   * @return An iterator pointing to the element with the smallest distance, a pointer to the end() if the landmakrs list is empty
   */
  std::vector<Map::single_landmark_s>::const_iterator _findClosestLandmark(const LandmarkObs& observation, const std::vector<Map::single_landmark_s>& landmarks);
};

#endif /* PARTICLE_FILTER_H_ */
