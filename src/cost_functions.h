/*
 * cost_functions.h
 *
 *  Created on: Jul 10, 2018
 *      Author: paolo
 */

#ifndef COST_FUNCTIONS_H_
#define COST_FUNCTIONS_H_


double time_diff_cost(vector<vector<double>> traj, int target_vehicle, vector<double> delta, double T, map<int, Vehicle> predictions);
    //"""
    //Penalizes trajectories that span a duration which is longer or
    //shorter than the duration requested.
    //"""

double s_diff_cost(vector<vector<double>> traj, int target_vehicle, vector<double> delta, double T, map<int, Vehicle> predictions);
    //"""
    //Penalizes trajectories whose s coordinate (and derivatives)
    //differ from the goal.
    //"""

double d_diff_cost(vector<vector<double>> traj, int target_vehicle, vector<double> delta, double T, map<int, Vehicle> predictions);
    //"""
    //Penalizes trajectories whose d coordinate (and derivatives)
    //differ from the goal.
    //"""

double collision_cost(vector<vector<double>> traj, int target_vehicle, vector<double> delta, double T, map<int, Vehicle> predictions);
    //"""
    //Binary cost function which penalizes collisions.
    //"""

double buffer_cost(vector<vector<double>> traj, int target_vehicle, vector<double> delta, double T, map<int, Vehicle> predictions);
    //"""
    //Penalizes getting close to other vehicles.
    //"""

//double stays_on_road_cost(vector<vector<double>> traj, int target_vehicle, vector<double> delta, double T, map<int, Vehicle> predictions);

//double exceeds_speed_limit_cost(vector<vector<double>> traj, int target_vehicle, vector<double> delta, double T, map<int, Vehicle> predictions);

double efficiency_cost(vector<vector<double>> traj, int target_vehicle, vector<double> delta, double T, map<int, Vehicle> predictions);
    //"""
    //Rewards high average speeds.
    //"""

double total_accel_cost(vector<vector<double>> traj, int target_vehicle, vector<double> delta, double T, map<int, Vehicle> predictions);
	//
	//
	//

double max_accel_cost(vector<vector<double>> traj, int target_vehicle, vector<double> delta, double T, map<int, Vehicle> predictions);
	//
	//
	//

double max_jerk_cost(vector<vector<double>> traj, int target_vehicle, vector<double> delta, double T, map<int, Vehicle> predictions);
	//
	//
	//

double total_jerk_cost(vector<vector<double>> traj, int target_vehicle, vector<double> delta, double T, map<int, Vehicle> predictions);
	//
	//
	//


#endif /* COST_FUNCTIONS_H_ */
