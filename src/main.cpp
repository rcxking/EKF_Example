/*
 * main.cpp
 *
 * Runner of an Extended Kalman Filter for a 2-Dimensional
 * planar robot.
 *
 * Bryant Pong
 * 6/9/19
 */

#include "WorldConstants.h"

#include <eigen3/Eigen/Dense>
#include <cassert>
#include <cstdlib>
#include <iostream>
#include <vector>

/**
 * CreateGroundTruth
 *
 * Given the 
 */
void CreateGroundTruth(const double linearVelocity,
                       const int numIterations,
                       const double linearVelocityCovariance,
                       const double angularVelocityCovariance,
                       const int trajectoryRadius,
                       std::vector<Eigen::Vector3d> *robotPoses,
                       std::vector<double> *nextLinearCommands,
                       std::vector<double> *nextAngularCommands,
                       std::vector<Eigen::Vector2d> *gpsReadings)
{
    // Ensure numIterations > 0
    if(numIterations <= 0)
        return;

    // Gaussian Distribution Random Numbers
    std::default_random_engine generator;
    std::normal_distribution<double> distribution;

    // Generate next linear and angular commands
    for(int i = 0; i < numIterations; ++i)
    {
        const double NEXT_LINEAR_COMMAND = linearVelocity +
          linearVelocityCovariance * distribution(generator);  

        const double NEXT_ANGULAR_COMMAND = linearVelocity / 
            trajectoryRadius + angularVelocityCovariance * 
            distribution(generator);

        nextLinearCommands->push_back(NEXT_LINEAR_COMMAND); 
        nextAngularCommands->push_back(NEXT_ANGULAR_COMMAND);

        std::cout << "Next linear command: " << NEXT_LINEAR_COMMAND << std::endl;
        std::cout << "Next angular command: " << NEXT_ANGULAR_COMMAND << std::endl;
    }

    // Generate the initial robot pose
    Eigen::Vector3d firstPose;
    firstPose << linearVelocity, 0, linearVelocity / trajectoryRadius;
    robotPoses->push_back(firstPose);

    // Generate the first GPS measurement
    Eigen::MatrixXd gpsMat(2, 3);
    gpsMat << 1, 0, 0, 0, 1, 0;

    Eigen::MatrixXd noise(2, 1);
    noise << distribution(generator), distribution(generator);
    Eigen::Vector2d firstGPSMeasurement = gpsMat * firstPose + noise;
    gpsReadings->push_back(firstGPSMeasurement);

    // Generate the remaining readings
    for(int i = 1; i < numIterations; ++i)
    {
        // Get the previous robot pose
        const Eigen::Vector3d PREV_POSE = robotPoses->at(i-1);

        // Next pose is the PREV_POSE + this step's movement
        Eigen::Vector3d nextStep;
        nextStep << cos(PREV_POSE(2)) * linearVelocity, sin(PREV_POSE(2)) * linearVelocity, linearVelocity / trajectoryRadius;

        const Eigen::Vector3d NEXT_POSE = PREV_POSE + nextStep;
        robotPoses->push_back(NEXT_POSE);

        // Create Gaussian Noise for GPS
        Eigen::MatrixXd gpsNoise(2, 1);
        gpsNoise << distribution(generator), distribution(generator);

        // Construct the next GPS measurement as a function of its current pose and added noise
        const Eigen::Vector2d NEXT_GPS = gpsMat * NEXT_POSE + gpsNoise; 
        gpsReadings->push_back(NEXT_GPS);
    } 

    // Ensure all data matrices are same length
    assert(nextLinearCommands->size() == nextAngularCommands->size());
    assert(robotPoses->size() == gpsReadings->size());
} 

int main(int argc, char **argv)
{
    // Usage:
    if(argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <number of timesteps>" << std::endl;
        return -1;
    }

    // Grab number of iterations to run the EKF filter
    const int NUM_ITERATIONS = atoi(argv[1]);

    std::cout << "Will run Extended Kalman Filter for: " << NUM_ITERATIONS <<
                    " iterations" << std::endl;

    /*
     * Velocity Covariance Matrix.  First column is for the linear
     * velocity noise.  Second column is for the angular velocity
     * noise.
     */
    Eigen::Matrix2d velocity_covariance_matrix;
    velocity_covariance_matrix << 
        (LINEAR_VELOCITY_COVARIANCE * LINEAR_VELOCITY_COVARIANCE), 0,
        0, (ANGULAR_VELOCITY_COVARIANCE * ANGULAR_VELOCITY_COVARIANCE);

    /*
     *  Initial state Covariance Matrix.  First column is for the robot's
     *  X-Coordinate.  Second column is the robot's Y-Coordinate.  Third
     *  column is the robot's orientation/heading.  The smaller the values,
     *  the more confident the robot is of its corresponding variable.
     */
    Eigen::Matrix3d initial_state_covariance_matrix;
    initial_state_covariance_matrix << 0.0001, 0, 0,
                                       0, 0.0001, 0,
                                       0, 0, 0.0001;

    std::cout << "Using Velocity Noise Covariance Matrix of:\n" <<
        velocity_covariance_matrix << std::endl;
    std::cout << "Initial State Covariance Matrix of:\n" <<
        initial_state_covariance_matrix << std::endl;

    /*
     * Given the robot's linear velocity, trajectory radius, noise covariances, and
     * the number of iterations, compute the ground truth of the robot pose
     * and create GPS data.     
     */
    
    // Robot Poses ground truth.  Of the form (x, y, theta (radians))
    std::vector<Eigen::Vector3d> robotPoses;

    // Linear and angular velocities at each timestamp
    std::vector<double> nextLinearCommands;
    std::vector<double> nextAngularCommands;

    // Simulated GPS Data.  Of the form (x, y)
    std::vector<Eigen::Vector2d> gpsReadings;

    // Populate the ground truth, motion commands, and GPS data 
    CreateGroundTruth(LINEAR_VELOCITY, NUM_ITERATIONS, LINEAR_VELOCITY_COVARIANCE,
          ANGULAR_VELOCITY_COVARIANCE, TRAJECTORY_RADIUS, &robotPoses,
          &nextLinearCommands, &nextAngularCommands, &gpsReadings);   

    return 0;
}
