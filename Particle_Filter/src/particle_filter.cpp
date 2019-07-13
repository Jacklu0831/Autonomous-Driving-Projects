/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include <sstream>

#include "helper_functions.h"

// I have a lazy soul
using namespace std;
// using std::string;
// using std::vector;

static default_random_engine gen;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  // return if the particle positions and rotations are already initialized
  if (is_initialized)
    return;

  num_particles = 10;
  // create normal distributions
  normal_distribution<double> dist_x(x, std[0]);
  normal_distribution<double> dist_y(y, std[1]);
  normal_distribution<double> dist_theta(theta, std[2]);

  // fill in random data for all particles
  for (int i=0; i<num_particles; i++){
    Particle p;
    p.id = i;
    p.x = dist_x(gen);
    p.x = dist_y(gen);
    p.theta = dist_theta(gen);
    p.weight = 1.0;

    // particles vector is used by main.cpp
    particles.push_back(p);
  }

  is_initialized = true;

  // function initialized member variables thus have no return value
}


void ParticleFilter::prediction(double delta_t, double std_pos[],
    double velocity, double yaw_rate) {
  /**
   * TODO: Add measurements to each particle and add random Gaussian noise.
   * NOTE: When adding noise you may find std::normal_distribution
   *   and std::default_random_engine useful.
   *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
   *  http://www.cplusplus.com/reference/random/default_random_engine/
   */

  for (size_t i = 0; i < num_particles; ++i) {

    // Gather old values
    double x_old = particles[i].x;
    double y_old = particles[i].y;
    double theta_old = particles[i].theta;

    double theta_pred, x_pred, y_pred;

    if (abs(yaw_rate) > 1e-5) {
      // Apply equations of motion model (turning)
      theta_pred = theta_old + yaw_rate * delta_t;
      x_pred = x_old + velocity / yaw_rate * (sin(theta_pred) - sin(theta_old));
      y_pred = y_old + velocity / yaw_rate * (cos(theta_old) - cos(theta_pred));
    } else {
      // Apply equations of motion model (going straight)
      theta_pred = theta_old;
      x_pred = x_old + velocity * delta_t * cos(theta_old);
      y_pred = y_old + velocity * delta_t * sin(theta_old);
    }

    normal_distribution<double> dist_x(x_pred, std_pos[0]);
    normal_distribution<double> dist_y(y_pred, std_pos[1]);
    normal_distribution<double> dist_theta(theta_pred, std_pos[2]);

    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
  }

}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
    vector<LandmarkObs>& observations) {

  for (auto& obs : observations) {
    double min_dist = numeric_limits<double>::max();

    for (const auto& pred_obs : predicted) {
      double d = dist(obs.x, obs.y, pred_obs.x, pred_obs.y);
      if (d < min_dist) {
        obs.id = pred_obs.id;
        min_dist = d;
      }
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
  const vector<LandmarkObs> &observations, const Map &map_landmarks) {

  double std_x = std_landmark[0];
  double std_y = std_landmark[1];

  for (size_t i = 0; i < num_particles; ++i) {

    double p_x = particles[i].x;
    double p_y = particles[i].y;
    double p_theta = particles[i].theta;

    // List all landmarks within sensor range
    vector<LandmarkObs> predicted_landmarks;

    for (const auto& map_landmark : map_landmarks.landmark_list) {
      int l_id = map_landmark.id_i;
      double l_x = (double) map_landmark.x_f;
      double l_y = (double) map_landmark.y_f;

      double d = dist(p_x, p_y, l_x, l_y);
      if (d < sensor_range) {
        LandmarkObs l_pred;
        l_pred.id = l_id;
        l_pred.x = l_x;
        l_pred.y = l_y;
        predicted_landmarks.push_back(l_pred);
      }
    }

    // List all observations in map coordinates
    vector<LandmarkObs> observed_landmarks_map_ref;
    for (size_t j = 0; j < observations.size(); ++j) {

      // Convert observation from particle(vehicle) to map coordinate system
      LandmarkObs rototranslated_obs;
      rototranslated_obs.x = cos(p_theta) * observations[j].x - sin(p_theta) * observations[j].y + p_x;
      rototranslated_obs.y = sin(p_theta) * observations[j].x + cos(p_theta) * observations[j].y + p_y;

      observed_landmarks_map_ref.push_back(rototranslated_obs);
    }

    dataAssociation(predicted_landmarks, observed_landmarks_map_ref);

    double particle_likelihood = 1.0;

    double mu_x, mu_y;
    for (const auto& obs : observed_landmarks_map_ref) {
      for (const auto& land : predicted_landmarks)
        if (obs.id == land.id) {
          mu_x = land.x;
          mu_y = land.y;
          break;
        }
      double norm_factor = 2 * M_PI * std_x * std_y;
      double prob = exp(-(pow(obs.x - mu_x, 2) / (2 * std_x * std_x) + pow(obs.y - mu_y, 2) / (2 * std_y * std_y)));

      particle_likelihood *= prob / norm_factor;
    }
    particles[i].weight = particle_likelihood;
  }
  double norm_factor = 0.0;
  for (const auto& particle : particles)
    norm_factor += particle.weight;

  // Normalize weights s.t. they sum to one
  for (Particle& particle : particles)
    particle.weight /= (norm_factor + numeric_limits<double>::epsilon());

}

void ParticleFilter::resample() {

  vector<double> particle_weights;
  for (Particle& particle : particles)
    particle_weights.push_back(particle.weight);

  discrete_distribution<int> weighted_distribution(particle_weights.begin(), particle_weights.end());

  vector<Particle> resampled_particles;
  for (int i = 0; i < num_particles; ++i) {
    int k = weighted_distribution(gen);
    resampled_particles.push_back(particles[k]);
  }

  particles = resampled_particles;

  for (Particle& particle : particles)
    particle.weight = 1.0;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // sense parameters, reset all
  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord) {
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1);  // get rid of the trailing space
  return s;
}