/*
 * helper_functions.h
 * Some helper functions for the 2D particle filter.
 *  Created on: Dec 13, 2016
 *      Author: Tiffany Huang
 */

#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include "map.h"
#include "Eigen/Dense"

/*
 * Struct representing one position/control measurement.
 */
struct control_s {
	
	double velocity;	// Velocity [m/s]
	double yawrate;		// Yaw rate [rad/s]
};

/*
 * Struct representing one ground truth position.
 */
struct ground_truth {
	
	double x;		// Global vehicle x position [m]
	double y;		// Global vehicle y position
	double theta;	// Global vehicle yaw [rad]
};

/*
 * Struct representing one landmark observation measurement.
 */
struct LandmarkObs {
	
	int id;				// Id of matching landmark in the map.
	double x;			// Local (vehicle coordinates) x position of landmark observation [m]
	double y;			// Local (vehicle coordinates) y position of landmark observation [m]
};

/*
 * Computes the Euclidean distance between two 2D points.
 * @param (x1,y1) x and y coordinates of first point
 * @param (x2,y2) x and y coordinates of second point
 * @output Euclidean distance between two 2D points
 */
inline double dist(double x1, double y1, double x2, double y2) {
	return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

inline double * getError(double gt_x, double gt_y, double gt_theta, double pf_x, double pf_y, double pf_theta) {
	static double error[3];
	error[0] = fabs(pf_x - gt_x);
	error[1] = fabs(pf_y - gt_y);
	error[2] = fabs(pf_theta - gt_theta);
	return error;
}

/* Reads map data from a file.
 * @param filename Name of file containing map data.
 * @output True if opening and reading file was successful
 */
inline bool read_map_data(std::string filename, Map& map) {

	// Get file of map:
	std::ifstream in_file_map(filename.c_str(),std::ifstream::in);
	// Return if we can't open the file.
	if (!in_file_map) {
		return false;
	}
	
	// Declare single line of map file:
	std::string line_map;

	// Run over each single line:
	while(getline(in_file_map, line_map)){

		std::istringstream iss_map(line_map);

		// Declare landmark values and ID:
		float landmark_x_f, landmark_y_f;
		int id_i;

		// Read data from current line to values::
		iss_map >> landmark_x_f;
		iss_map >> landmark_y_f;
		iss_map >> id_i;

		// Declare single_landmark:
		Map::single_landmark_s single_landmark_temp;

		// Set values
		single_landmark_temp.id_i = id_i;
		single_landmark_temp.x_f  = landmark_x_f;
		single_landmark_temp.y_f  = landmark_y_f;

		// Add to landmark list of map:
		map.landmark_list.push_back(single_landmark_temp);
	}
	return true;
}

/* Reads control data from a file.
 * @param filename Name of file containing control measurements.
 * @output True if opening and reading file was successful
 */
inline bool read_control_data(std::string filename, std::vector<control_s>& position_meas) {

	// Get file of position measurements:
	std::ifstream in_file_pos(filename.c_str(),std::ifstream::in);
	// Return if we can't open the file.
	if (!in_file_pos) {
		return false;
	}

	// Declare single line of position measurement file:
	std::string line_pos;

	// Run over each single line:
	while(getline(in_file_pos, line_pos)){

		std::istringstream iss_pos(line_pos);

		// Declare position values:
		double velocity, yawrate;

		// Declare single control measurement:
		control_s meas;

		//read data from line to values:

		iss_pos >> velocity;
		iss_pos >> yawrate;

		
		// Set values
		meas.velocity = velocity;
		meas.yawrate = yawrate;

		// Add to list of control measurements:
		position_meas.push_back(meas);
	}
	return true;
}

/* Reads ground truth data from a file.
 * @param filename Name of file containing ground truth.
 * @output True if opening and reading file was successful
 */
inline bool read_gt_data(std::string filename, std::vector<ground_truth>& gt) {

	// Get file of position measurements:
	std::ifstream in_file_pos(filename.c_str(),std::ifstream::in);
	// Return if we can't open the file.
	if (!in_file_pos) {
		return false;
	}

	// Declare single line of position measurement file:
	std::string line_pos;

	// Run over each single line:
	while(getline(in_file_pos, line_pos)){

		std::istringstream iss_pos(line_pos);

		// Declare position values:
		double x, y, azimuth;

		// Declare single ground truth:
		ground_truth single_gt; 

		//read data from line to values:
		iss_pos >> x;
		iss_pos >> y;
		iss_pos >> azimuth;

		// Set values
		single_gt.x = x;
		single_gt.y = y;
		single_gt.theta = azimuth;

		// Add to list of control measurements and ground truth:
		gt.push_back(single_gt);
	}
	return true;
}

/* Reads landmark observation data from a file.
 * @param filename Name of file containing landmark observation measurements.
 * @output True if opening and reading file was successful
 */
inline bool read_landmark_data(std::string filename, std::vector<LandmarkObs>& observations) {

	// Get file of landmark measurements:
	std::ifstream in_file_obs(filename.c_str(),std::ifstream::in);
	// Return if we can't open the file.
	if (!in_file_obs) {
		return false;
	}

	// Declare single line of landmark measurement file:
	std::string line_obs;

	// Run over each single line:
	while(getline(in_file_obs, line_obs)){

		std::istringstream iss_obs(line_obs);

		// Declare position values:
		double local_x, local_y;

		//read data from line to values:
		iss_obs >> local_x;
		iss_obs >> local_y;

		// Declare single landmark measurement:
		LandmarkObs meas;

		// Set values
		meas.x = local_x;
		meas.y = local_y;

		// Add to list of control measurements:
		observations.push_back(meas);
	}
	return true;
}
/* normalize angle to be in range of [0,2pi].
 * @param angle angle to be normalized.
 * @output normalized angle
 */
inline double normalize_angle(double angle) {
    if (angle > 2*M_PI){
        angle = fmod((angle - 2*M_PI),(2*M_PI));
    }
    if (angle < 0){
        angle = fmod((angle + 2*M_PI),(2*M_PI));
    }
    return angle;
}
/* get mvnpdf
 * @param x location for pdf
 * @param mu, covariance parameter for mvn
 * @output mvnpdf
 */
inline double mvnpdf(Eigen::VectorXd x, Eigen::VectorXd mu, Eigen::MatrixXd covariance) {
    Eigen::VectorXd center = x-mu;
    int dimension = mu.size();
    double maha_dist = center.transpose()*covariance.inverse()*center;
    double covariance_norm = covariance.determinant();
    double pdf = pow(2*M_PI, -dimension/2)*pow(covariance_norm,-0.5)*exp(-0.5*maha_dist);
    return pdf;
}
/* get mvnpdf for independent components
 * @param x location for pdf
 * @param mu, covariance parameter for mvn under independence assumption
 * @output mvnpdf
 */
inline double mvnpdf(std::vector<double> x, std::vector<double> mu,  std::vector<double> sigma) {
    int dimension = mu.size();
    double* x_temp = &x[0];
    double* mu_temp = &mu[0];
    double* sigma_temp = &sigma[0];
    Eigen::Map<Eigen::VectorXd> x_eigen(x_temp,dimension);
    Eigen::Map<Eigen::VectorXd> mu_eigen(mu_temp,dimension);
    Eigen::Map<Eigen::VectorXd> sigma_eigen(sigma_temp,dimension);
    sigma_eigen.array().pow(2);
    Eigen::MatrixXd covariance = sigma_eigen.asDiagonal();

    Eigen::VectorXd center = x_eigen-mu_eigen;
    double maha_dist = center.transpose()*covariance.inverse()*center;
    double covariance_norm = covariance.determinant();
    double pdf = pow(2*M_PI, -dimension/2)*pow(covariance_norm,-0.5)*exp(-0.5*maha_dist);
    return pdf;
}

/* transform local coordinate to global coordinate
 * @param ego_x,ego_y origin location for local cooridnate
 * @param ego_theta angle for local coordinate
 * @param local local cooridnate to be transformed, struct LandmarkObs
 * @output global  LandmarkObs in global coordinate
 */
inline LandmarkObs coordinate_transform(LandmarkObs local, double ego_x, double ego_y, double ego_theta){
    LandmarkObs global;
    global.id = local.id;
    global.x = local.x*cos(ego_theta)-local.y*sin(ego_theta)+ego_x;
    global.y = local.x*sin(ego_theta)+local.y*cos(ego_theta)+ego_y;
    return  global;
}
#endif /* HELPER_FUNCTIONS_H_ */
