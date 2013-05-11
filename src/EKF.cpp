/*
 * EKF.h
 *
 *  Created on: May 7, 2012
 *      Author: engelhan
 */

#include "ex_2/EKF.h"

using namespace std;
using namespace Eigen;


 // odometry:
 // x: distance traveled in local x-direction
 // y: distance traveled in local y-direction
 // phi: rotation update
 void ExtendedKalmanFilter::predictionStep(const Eigen::Vector3f& odometry)
 {

	state(0) = state(0) + cos(state(2))*odometry(0) - sin(state(2))*odometry(1);
	state(1) = state(1) + sin(state(2))*odometry(0) + cos(state(2))*odometry(1);
	state(2) = state(2) + odometry(2);

	state(2) = atan2(sin(state(2)),cos(state(2))); // normalize angle

	// dg/dx:
	Eigen::Matrix3f G;

	G << 1, 0, -sin(state(2))*odometry(0) - cos(state(2))*odometry(1),
		0, 1,  cos(state(2))*odometry(0) - sin(state(2))*odometry(1),
		0, 0,  1;

	sigma = G*sigma*G.transpose() + Q;
 }






/* ==================== TO IMPLEMENT =======================
 * measurement(0) : x-position of marker in drone's xy-coordinate system (independant of roll, pitch)
 * measurement(1): y-position of marker in drone's xy-coordinate system (independant of roll, pitch)
 * measurement(2): yaw rotation of marker, in drone's xy-coordinate system (independant of roll, pitch)
 *
 * global_marker_pose(0): x-position or marker in world-coordinate system
 * global_marker_pose(1): y-position or marker in world-coordinate system
 * global_marker_pose(2): yaw-rotation or marker in world-coordinate system
 */
 void ExtendedKalmanFilter::correctionStep(const Eigen::Vector3f& measurement, const Eigen::Vector3f& global_marker_pose)
 {
	printf("TO IMPLEMENT! ekf: %f %f %f;    obs: %f %f %f\n",
			state[0],state[1],state[2],
			measurement[0], measurement[1], measurement[2]);
}



void ExtendedKalmanFilter::initFilter()
{
	state =  Eigen::Vector3f(0,0,0);
	sigma = Eigen::Matrix3f::Zero(); sigma(0,0) = sigma(1,1) = 1; sigma(2,2) = 1;
	Q = Eigen::Matrix3f::Zero();     Q(0,0) = 0.0006; Q(1,1) = 0.0003; Q(2,2) = 0.0001;
	R = Eigen::Matrix3f::Zero();     R(0,0) = R(1,1) = 0.3; R(2,2) = 0.1;
}


