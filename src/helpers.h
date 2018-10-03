/*
 * helpers.h
 *
 *  Created on: Jul 10, 2018
 *      Author: paolo
 */

#ifndef HELPERS_H_
#define HELPERS_H_
#include <vector>
#include <map>
#include "vehicle.h"

using namespace std;


// For converting back and forth between radians and degrees.
constexpr double pi() ;

double deg2rad(double x);

double rad2deg(double x);

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

double f(vector<double> c, double t);

double logistic(double x);

/*vector<double> differentiate(const vector<double> & coefficients);

double nearest_approach_to_any_vehicle(vector<vector<double>> traj, map<int,Vehicle> vehicles);
double nearest_approach(vector<vector<double>> traj, Vehicle vehicle);

vector<vector<double>> get_f_and_N_derivatives(vector<double> coeffs, int N=3);
*/
vector<double> JMT(vector<double> start, vector<double> end, double T);
#endif /* HELPERS_H_ */
