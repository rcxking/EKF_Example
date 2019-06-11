#ifndef _WORLD_CONSTANTS_H_
#define _WORLD_CONSTANTS_H_

/*
 * WorldConstants.h
 *
 * Contains tuneable constants that describe robot and world properties
 * used in the Extended Kalman Filter.
 *
 * Bryant Pong
 * 6/9/19
 */    

#include <cmath>

/*
 * Trajectory constants
 *
 * The robot will be moving in an elliptical pattern.
 */ 
const int    TRAJECTORY_RADIUS = 5;
const double LINEAR_VELOCITY   = 0.1;
const double ROTATION_VELOCITY = 0.02;

/*
 * Covariances of linear and angular velocity.  Higher values mean
 * there is more uncertainty involved.    
 */
const double LINEAR_VELOCITY_COVARIANCE = 0.01 * LINEAR_VELOCITY;
const double ANGULAR_VELOCITY_COVARIANCE = (0.02 * M_PI) / 180.0;

#endif
