/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>

#include "particle_filter.h"

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
	std::default_random_engine gen;
	std::normal_distribution<double> dist_x(x, std[0]);
	std::normal_distribution<double> dist_y(y, std[1]);
	std::normal_distribution<double> dist_theta(theta, std[2]);
	weights.resize(num_particles,0);
	particles.resize(num_particles,0);
	for(int particle_id = 0; particle_id < num_particles; particle_id++){
		Particle P;
		P.id = particle_id;
		P.x = dist_x(gen);
		P.y = dist_y(gen);
		P.theta = normalize_angle(dist_theta(gen));
		P.weight = 1;
		weights[particle_id] = 1;
		particles[particle_id] = P;
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/
	std::default_random_engine gen;
    std::normal_distribution<double> dist_x(0, std_pos[0]);
    std::normal_distribution<double> dist_y(0, std_pos[1]);
    std::normal_distribution<double> dist_theta(0, std_pos[2]);
    for(int particle_id = 0; particle_id < num_particles; particle_id++){
        double x = particles[particle_id].x;
        double y = particles[particle_id].y;
        double theta = particles[particle_id].theta;

        double x_update, y_update, theta_update;
        if(abs(yaw_rate) < 0.0001){
            x_update = x + velocity*delta_t;
            y_update = y + velocity*delta_t;
            theta_update = theta;
        }else{
            theta_update = theta + yaw_rate*delta_t;
            x_update = x + velocity/yaw_rate*(sin(theta_update)-sin(theta));
            y_update = y + velocity/yaw_rate*(cos(theta)-cos(theta_update));
        }
        particles[particle_id].x = x_update+dist_x(gen);
        particles[particle_id].y = y_update+dist_y(gen);
        particles[particle_id].theta = normalize_angle(theta_update+dist_theta(gen));
    }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
    std::vector<LandmarkObs> observations_reorder;
    observations_reorder.resize(observations.size(),0);
    for(int prediction_id=0; prediction_id < predicted.size(); prediction_id++){
        LandmarkObs prediction = predicted[prediction_id];
        LandmarkObs observation = observations[0];
        observations_reorder[prediction_id] = observation;

        double distance_current = dist(prediction.x,prediction.y,observation.x,observation.y);
        for(int observation_id=1; observation_id < observations.size(); observation_id++){
            observation = observations[observation_id];
            double distance = dist(prediction.x,prediction.y,observation.x,observation.y);
            if(distance < distance_current){
                observations_reorder[prediction_id] = observation;
            }
        }
    }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		std::vector<LandmarkObs> observations, Map map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33. Note that you'll need to switch the minus sign in that equation to a plus to account 
	//   for the fact that the map's y-axis actually points downwards.)
	//   http://planning.cs.uiuc.edu/node99.html
    double critical_distance = sensor_range + 10;
    for(int particle_id = 0; particle_id < particles.size(); particle_id++){
        double ego_x = particles[particle_id].x;
        double ego_y = particles[particle_id].y;
        double ego_theta = particles[particle_id].theta;
        for(int land_mark_id = 0; land_mark_id < map_landmarks.landmark_list.size(); land_mark_id++){

        }
    }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

}

void ParticleFilter::write(std::string filename) {
	// You don't need to modify this file.
	std::ofstream dataFile;
	dataFile.open(filename, std::ios::app);
	for (int i = 0; i < num_particles; ++i) {
		dataFile << particles[i].x << " " << particles[i].y << " " << particles[i].theta << "\n";
	}
	dataFile.close();
}
