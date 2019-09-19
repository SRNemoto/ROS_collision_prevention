#ifndef CONSTANTS_H
#define CONSTANTS_H

#define _USE_MATH_DEFINES
#include <string>
#include <math.h>

std::string laser_topic("laser_1");

// Default input velocity topic name "des_vel"
std::string input_vel_topic("des_vel");

// Default output topic name "cmd_vel"
std::string output_vel_topic("cmd_vel");

const float ROBOT_RADIUS = 0.15; // meters

const float MIN_DISTANCE = 0.5; //meters

const float MIN_LASER_READING = ROBOT_RADIUS + MIN_DISTANCE; // minimum lidar reading in meters

// Angle converted to radians and converted to integer
const int ANGLE_MARGIN = static_cast<int>(ceil(tan(ROBOT_RADIUS / (MIN_LASER_READING)) * (180 / M_PI)));

#endif