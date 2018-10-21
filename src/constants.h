/*
 * Constants.h
 *
 *  Created on: Jul 10, 2018
 *      Author: paolo
 */
#ifndef CONSTANTS_H_
#define CONSTANTS_H_
#include <vector>

#define DEBUG 0

double const VEHICLE_RADIUS = 2;// model vehicle as circle to simplify collision detection

int const WAYPOINTSSIZE = 50;
int const EGO_ID = 0;

double const TIMESTEP = 9; // seconds
double const WAYPOINTQUANT = 0.02;
int const TRAJECTORYSTEPS = 25;
double const in_range_dist = 65;
int const KL_BUFFER = 25;
double const MPHTOMS = 0.44704;


const float REACH_GOAL = pow(10, 1);
const float EFFICIENCY = pow(10, 1);
const float COLLISION  = pow(10, 2);
const float BUFFER     = pow(10, 1);

const double max_s = 6945.554;


#endif /* CONSTANTS_H_ */
