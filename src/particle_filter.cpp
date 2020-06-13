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

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */

  // Set the number of particles
  num_particles = 30;  

  // Resize weights and particle vectors based on num_particles
  weights.resize(num_particles);
  particles.resize(num_particles);
  
  // Engine for later generation of particles
  std::default_random_engine gen;
    
  // Creates a normal (Gaussian) distribution for x, y and theta (yaw) for noise purposes
  std::normal_distribution<double> dist_x(x, std[0]);
  std::normal_distribution<double> dist_y(y, std[1]);
  std::normal_distribution<double> dist_theta(theta, std[2]);
    
  // Initializes particles, we apply the uniform distribution
  for (int i = 0; i < num_particles; ++i) {
      
    // Add generated particle data to particles class
    particles[i].id = i;
    particles[i].x = dist_x(gen);
    particles[i].y = dist_y(gen);
    particles[i].theta = dist_theta(gen);
    particles[i].weight = 1.0;
   
   // Add initial weight to weights vector
    weights[i] = 1.0;

  }
    
  // Change the flag to initialized
  is_initialized = true;

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

  // Engine for later generation of particles
  std::default_random_engine gen;

  // Make normal distributions for noise
  std::normal_distribution<double> dist_x(0, std_pos[0]);
  std::normal_distribution<double> dist_y(0, std_pos[1]);
  std::normal_distribution<double> dist_theta(0, std_pos[2]);

  // Check for the yaw, if it is zero the equations are different
  for (int i = 0; i < num_particles; ++i) {
    
    if (fabs(yaw_rate) > 0.001) {
      // Add measurements to a particle
      particles[i].x += (velocity/yaw_rate) * (sin(particles[i].theta + (yaw_rate * delta_t)) - sin(particles[i].theta));
      particles[i].y += (velocity/yaw_rate) * (cos(particles[i].theta) - cos(particles[i].theta + (yaw_rate * delta_t)));
      particles[i].theta += yaw_rate * delta_t;
      
    } else {
      // Add measurements to a particle
      particles[i].x += velocity * delta_t * cos(particles[i].theta);
      particles[i].y += velocity * delta_t * sin(particles[i].theta);
      
      // Theta will stay the same due to no yaw_rate
    }

    // Add noise to the particle
    particles[i].x += dist_x(gen);
    particles[i].y += dist_y(gen);
    particles[i].theta += dist_theta(gen);
  }
  
}


void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations) {
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

  // Associate observations in map coordinates to predicted landmarks using nearest neighbor algorithm. It's  important to note 
  // that the number of observations may be less than the total number of landmarks as some of the landmarks may be outside 
  // the range of vehicle's sensor.
  for (unsigned int i = 0; i < observations.size(); i++) {
  // Set minimum distance as a big number for further updates
    double min_dist = std::numeric_limits<double>::max();

    // Id of landmark from map to be associated with the observation
    int closest_landmark_id = -1;

    // Get current observation
    LandmarkObs obs = observations[i];

    for (unsigned int j = 0; j < predicted.size(); j++) {
      // Get current prediction
      LandmarkObs pred = predicted[j];

      // Calculate distance
      double current_dist = dist(obs.x, obs.y, pred.x, pred.y);

      // Check for minimum distance
      if (current_dist < min_dist) {
        min_dist = current_dist;
        closest_landmark_id = pred.id;
      }

    }

    // Set landmark to current observation
    observations[i].id = closest_landmark_id;
  }

}


void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations, 
                                   const Map &map_landmarks) {
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

  // Variable to normalize weights [0-1]
  double weight_normalizer = 0.0;

  // Cached values of multivariate gaussian distribution for later use
  double sigma_x = std_landmark[0];
  double sigma_y = std_landmark[1];
  double gauss_denom_x = 2.0 * pow(sigma_x, 2);
  double gauss_denom_y = 2.0 * pow(sigma_y, 2);
  double normalizer = 1.0 / (2.0 * M_PI * sigma_x * sigma_y);

  for (int i = 0; i < num_particles; i++) {
    // Get current particle
    Particle particle = particles[i];

    // Step 1: Transform observations from vehicle coordinates to map coordinates.

    // Vector to store all the transformed observations
    vector<LandmarkObs> transformed_observations;

    for (unsigned int j = 0; j < observations.size(); j++) {
      // Get current observation
      LandmarkObs obs = observations[j];

      // Calculate transformation for current observation
      LandmarkObs transformed_obs;

      transformed_obs.x = particle.x + (cos(particle.theta) * obs.x) - (sin(particle.theta) * obs.y);
      transformed_obs.y = particle.y + (sin(particle.theta) * obs.x) + (cos(particle.theta) * obs.y);
      transformed_obs.id = j;

      // Append observation to observations vector
      transformed_observations.push_back(transformed_obs);

    }

    // Step 2: Filter map landmarks to keep only those which are in the sensor_range of current particle. Push them to predictions vector.

    vector<LandmarkObs> predicted_landmarks;

    for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); j++) {
      // Get current landmark
      Map::single_landmark_s current_landmark = map_landmarks.landmark_list[j];

      // Check if landmark in sensor range
      if ( fabs(particle.x - current_landmark.x_f) <= sensor_range && fabs(particle.y - current_landmark.y_f) <= sensor_range ) {
        predicted_landmarks.push_back(LandmarkObs {current_landmark.id_i, current_landmark.x_f, current_landmark.y_f});
      }

    }

    // Step 3: Associate observations to predicted landmarks using nearest neighbor algorithm.

    //Associate observations with predicted landmarks
    dataAssociation(predicted_landmarks, transformed_observations);

    // Step 4: Calculate the weight of each particle using Multivariate Gaussian distribution.

    // Reset the weight of particle to 1.0
    particles[i].weight = 1.0;

    // Calculate the weight of particle based on the multivariate Gaussian probability function*/
    for (unsigned int j = 0; j < transformed_observations.size(); j++) {
      // Get current transformed observation
      LandmarkObs transformed_obs;
      transformed_obs = transformed_observations[j];

      // Multi-Variate Gaussian distribution of each observation, for each particle
      double multi_prob = 1.0;

      for (unsigned int k = 0; k < predicted_landmarks.size(); k++) {
        // Get current prediction landmark
        LandmarkObs predicted_landmark;
        predicted_landmark = predicted_landmarks[k];

        if (transformed_obs.id == predicted_landmark.id) {
          // Compute exponent (pow(xm-mu_x,2)/gauss_den_x + pow(ym-mu_y,2)/gauss_den_y)
          double exponent = pow(transformed_obs.x - predicted_landmark.x, 2) / gauss_denom_x + pow(transformed_obs.y - predicted_landmark.y, 2) / gauss_denom_y;
          // This will be the weight
          multi_prob *= normalizer * exp(-exponent);
          // Update particle weight
          particles[i].weight = multi_prob;
        }

      }
    }

    // Add weight to the normalize
    weight_normalizer += particles[i].weight;

  }

  // Step 5: Normalize the weights of all particles since resmapling using probabilistic approach.
  for (unsigned int i = 0; i < particles.size(); i++) {
    particles[i].weight /= weight_normalizer;
    weights[i] = particles[i].weight;
  }

}


void ParticleFilter::resample() {
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  vector<Particle> resampled_particles;

  // Create a generator to be used for generating random particle index and beta value
  std::default_random_engine gen;
  
  // Generate random particle index
  std::uniform_int_distribution<int> particle_index(0, num_particles - 1);
  
  // Initialize with a random index
  int current_index = particle_index(gen);
  
  // Initialise beta
  double beta = 0.0;
  
  double max_weight_2 = 2.0 * *max_element(weights.begin(), weights.end());
  
  // Resample wheel algorithm
  for (unsigned int i = 0; i < particles.size(); i++) {
    std::uniform_real_distribution<double> random_weight(0.0, max_weight_2);
    beta += random_weight(gen);

    while (beta > weights[current_index]) {
      beta -= weights[current_index];
      current_index = (current_index + 1) % num_particles;
    }
    resampled_particles.push_back(particles[current_index]);
  }
  particles = resampled_particles;

}


void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y) {
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best) {
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
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
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}