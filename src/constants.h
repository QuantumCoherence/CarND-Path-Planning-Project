/*
 * Constants.h
 *
 *  Created on: Jul 10, 2018
 *      Author: paolo
 */

#ifndef CONSTANTS_H_
#define CONSTANTS_H_
#include <vector>

int const N_SAMPLES = 10;
std::vector<double> const SIGMA_S = {10.0, 4.0, 2.0};   // s, s_dot, s_double_dot
std::vector<double> const SIGMA_D = {1.0, 1.0, 1.0};
double const SIGMA_T = 2.0;

double const MAX_JERK = 10 ;   // m/s/s/s
double const MAX_ACCEL= 10;    // m/s/s

double const EXPECTED_JERK_IN_ONE_SEC = 2;   // m/s/s
double const EXPECTED_ACC_IN_ONE_SEC = 1;    // m/s

double const SPEED_LIMIT = 30;
double const VEHICLE_RADIUS =2;// model vehicle as circle to simplify collision detection

int const WAYPOINTSSIZE = 50;
int const EGO_ID = 0;

double const TIMESTEP = 9; // seconds
double const WAYPOINTQUANT = 0.02;
int const TRAJECTORYSTEPS = 12;
double const in_range_dist = 60;
double const MPHTOMS = 0.44704;


const float REACH_GOAL = pow(10, 1);
const float EFFICIENCY = pow(10, 3);
const float COLLISION  = pow(10, 5);
const float BUFFER     = pow(10, 3);

const double max_s = 6945.554;


#endif /* CONSTANTS_H_ */
