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

void ParticleFilter::init(double x, double y, double theta, std::vector<double> std) {

	std::default_random_engine gen;
	std::normal_distribution<double> dist_x(x, std[0]);
	std::normal_distribution<double> dist_y(y, std[1]);
	std::normal_distribution<double> dist_theta(theta, std[2]);

	for(int particle_id = 0; particle_id < num_particles; particle_id++){
		Particle P;
		P.id = particle_id;
		P.x = dist_x(gen);
		P.y = dist_y(gen);
		P.theta = normalize_angle(dist_theta(gen));
		P.weight = 1;
		weights.push_back(1);
		particles.push_back(P);
	}
	is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, std::vector<double> std_pos, double velocity, double yaw_rate) {

	std::default_random_engine gen;
    std::normal_distribution<double> dist_x(0, std_pos[0]);
    std::normal_distribution<double> dist_y(0, std_pos[1]);
    std::normal_distribution<double> dist_theta(0, std_pos[2]);
    for(int particle_id = 0; particle_id < num_particles; particle_id++){
        double x = particles[particle_id].x;
        double y = particles[particle_id].y;
        double theta = particles[particle_id].theta;

        double x_update, y_update, theta_update;
//        update mean pose
        if(abs(yaw_rate) < 0.00001){
            x_update = x + velocity*delta_t;
            y_update = y + velocity*delta_t;
            theta_update = theta;
        }else{
            theta_update = theta + yaw_rate*delta_t;
            x_update = x + velocity/yaw_rate*(sin(theta_update)-sin(theta));
            y_update = y + velocity/yaw_rate*(cos(theta)-cos(theta_update));
        }
//        add motion noise
        particles[particle_id].x = x_update+dist_x(gen);
        particles[particle_id].y = y_update+dist_y(gen);
        particles[particle_id].theta = normalize_angle(theta_update+dist_theta(gen));
    }
}

double ParticleFilter::dataAssociation(Map map_landmarks, std::vector<LandmarkObs>& observations,
                                     std::vector<LandmarkObs>& observations_match, std::vector<LandmarkObs>& landmark_match,
                                     std::vector<double>& distance_x, std::vector<double>& distance_y,
                                     std::vector<double>& weight, std::vector<double> std_landmark,
                                     double self_x, double self_y, double self_theta, double critical_distance) {
    int observation_match_id = 0;
    double total_weight = 0;

    for(int landmark_id=0; landmark_id < map_landmarks.landmark_list.size(); landmark_id++){

        double land_mark_x = map_landmarks.landmark_list[landmark_id].x_f;
        double land_mark_y = map_landmarks.landmark_list[landmark_id].y_f;
        double distance_to_host = dist(self_x, self_y, land_mark_x, land_mark_y);
        std::vector<double> land_mark_location;
        land_mark_location.push_back(land_mark_x);
        land_mark_location.push_back(land_mark_y);
//      check landmark within range
        if(distance_to_host<(critical_distance)){
            if(observation_match_id==0){total_weight = 1;}

            LandmarkObs observation = observations[0];
            observation = coordinate_transform(observation, self_x, self_y, self_theta);
            std::vector<double> observation_location;
            observation_location.push_back(observation.x);
            observation_location.push_back(observation.y);
            double weight_location = mvnpdf(observation_location, land_mark_location, std_landmark);

            observations_match.push_back(observation);
            distance_x.push_back(observation.x - land_mark_x);
            distance_y.push_back(observation.y - land_mark_y);
            weight.push_back(weight_location);
// get closest measurement
            double distance_current = dist(land_mark_x,land_mark_y,observation.x,observation.y);
            for(int observation_id=1; observation_id < observations.size(); observation_id++){
                observation = observations[observation_id];
                observation = coordinate_transform(observation, self_x, self_y, self_theta);

                double distance = dist(land_mark_x,land_mark_y,observation.x,observation.y);
                if(distance < distance_current){
                    observation_location[0] = observation.x;
                    observation_location[1] = observation.y;
//                    update weight based on mvn
                    weight_location = mvnpdf(observation_location, land_mark_location, std_landmark);

                    observations_match.back() = observation;
                    distance_x.back() = observation.x - land_mark_x;
                    distance_y.back() = observation.y - land_mark_y;
                    weight.back() = weight_location;

                    distance_current = distance;
                }
            }
            observation_match_id++;
            total_weight *=  weight.back();
        }
    }
    return total_weight;
}

void ParticleFilter::updateWeights(double sensor_range, std::vector<double> std_landmark,
                                   std::vector<LandmarkObs> observations, Map map_landmarks) {

    double critical_distance = sensor_range;
    double sum_weight = 0.0;
    for(int particle_id = 0; particle_id < particles.size(); particle_id++){
        double ego_x = particles[particle_id].x;
        double ego_y = particles[particle_id].y;
        double ego_theta = particles[particle_id].theta;
        std::vector<LandmarkObs> observations_match, landmark_match;
        std::vector<double> distance_x, distance_y, weight_particle;
        particles[particle_id].weight = dataAssociation(map_landmarks, observations,
                                                        observations_match, landmark_match,
                                                        distance_x, distance_y,
                                                        weight_particle, std_landmark,
                                                        ego_x, ego_y, ego_theta, critical_distance);
        weights[particle_id] = particles[particle_id].weight;
        sum_weight += weights[particle_id];
    }
//    normalize weight to get discrete pdf
    for(int particle_id = 0; particle_id < particles.size(); particle_id++){
        particles[particle_id].weight /= sum_weight;
        weights[particle_id] /= sum_weight;
    }
}

void ParticleFilter::resample() {
    std::discrete_distribution<int> dist_particle_weight (weights.begin(),weights.end());
    std::default_random_engine gen;
    std::vector<Particle> particles_temp;
    for(int particle_id = 0; particle_id < num_particles; particle_id++){
        int sample_particle_id = dist_particle_weight(gen);
        particles_temp.push_back(particles[sample_particle_id]);
        particles_temp[particle_id].id = particle_id;
    }
    particles = particles_temp;
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
