/*
 * cost_functions.cpp
 *
 *  Created on: Jul 10, 2018
 *      Author: paolo
 */
#include "helpers.h"
#include <vector>
#include <iostream>
#include <map>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "cost_functions.h"
#include "constants.h"
#include "helpers.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;



// COST FUNCTIONS
double time_diff_cost(vector<vector<double>> traj, int target_vehicle, vector<double> delta, double T, map<int, Vehicle> predictions){
    //"""
    //Penalizes trajectories that span a duration which is longer or
    //shorter than the duration requested.
    //"""
    double t = traj[2][0];
    return logistic(float(abs(t-T)) / T);
}

double s_diff_cost(vector<vector<double>> traj, int target_vehicle, vector<double> delta, double T, map<int, Vehicle> predictions){
    //"""
    //Penalizes trajectories whose s coordinate (and derivatives)
    //differ from the goal.
    //"""
    vector<double> s = traj[0];
	double t = traj[2][0];
    vector<double> target = predictions[target_vehicle].state_in(t);
    for (int i=0; i<target.size(); i++){
    	target[i] = target[i] + delta[i];
    }
    vector<double> s_targ;
	for (int i=0; i<3; i++){
		s_targ[i] = target[i];
	}
	vector<vector<double>> coefs_D = get_f_and_N_derivatives(s, 2);
    vector<double> S;
	for (int i=0; i<coefs_D.size(); i++){
		S[i] = f(coefs_D[i], t);
	}
    double cost = 0.0;
    int steps;
    if(S.size()<=s_targ.size()) steps = S.size();
    else steps = s_targ.size();
    if(SIGMA_S.size()<=steps) steps = SIGMA_S.size();
    for (int i=0; i<steps;i++) {
    	double diff = abs(S[i]-s_targ[i]);
    	cost+= logistic(diff/SIGMA_S[i]);
    }
    return cost;
}

double d_diff_cost(vector<vector<double>> traj, int target_vehicle, vector<double> delta, double T, map<int, Vehicle> predictions){
    //"""
    //Penalizes trajectories whose d coordinate (and derivatives)
    //differ from the goal.
    //"""
	vector<double> d = traj[1];
	double t = traj[2][0];

	vector<vector<double>> coefs_D = get_f_and_N_derivatives(d, 2);
    vector<double> D;
	for (int i=0; i<coefs_D.size(); i++){
		D[i] = f(coefs_D[i], t);
	}
    double cost = 0.0;
    int steps;

	vector<double> target = predictions[target_vehicle].state_in(t);
	for (int i=0; i<target.size(); i++){
		target[i] = target[i] + delta[i];
	}
	vector<double> d_targ;
	for (int i=3; i<6; i++){
		d_targ[i] = target[i];
	}

	if(D.size()<=d_targ.size()) steps = D.size();
	else steps = d_targ.size();
	if(SIGMA_S.size()<=steps) steps = SIGMA_S.size();
	for (int i=0; i<steps;i++) {
		double diff = abs(D[i]-d_targ[i]);
		cost+= logistic(diff/SIGMA_S[i]);
	}
    return cost;
}

double collision_cost(vector<vector<double>> traj, int target_vehicle, vector<double> delta, double T, map<int, Vehicle> predictions){
    //"""
    //Binary cost function which penalizes collisions.
    //"""

    double nearest = nearest_approach_to_any_vehicle(traj, predictions);
    if (nearest < 2*VEHICLE_RADIUS) return 1.0;
    else return 0.0;
}

double buffer_cost(vector<vector<double>> traj, int target_vehicle, vector<double> delta, double T, map<int, Vehicle> predictions){
    //"""
    //Penalizes getting close to other vehicles.
    //"""

	double nearest = nearest_approach_to_any_vehicle(traj, predictions);
    return logistic(2*VEHICLE_RADIUS / nearest);
}

/*
double stays_on_road_cost(vector<vector<double>> traj, int target_vehicle, vector<double> delta, double T, map<int, Vehicle> predictions){
    return 0.0;
}

double exceeds_speed_limit_cost(vector<vector<double>> traj, int target_vehicle, vector<double> delta, double T, map<int, Vehicle> predictions){
	return 0.0;
}
*/
double efficiency_cost(vector<vector<double>> traj, int target_vehicle, vector<double> delta, double T, map<int, Vehicle> predictions){
    //"""
    //Rewards high average speeds.
    //"""
    vector<double> s = traj[0];
	double t = traj[2][0];
    double avg_v = f(s,t)/t;
	vector<double> target = predictions[target_vehicle].state_in(t);
	double targ_s= target[0];
    double targ_v = targ_s/t;
    return logistic(2*(targ_v - avg_v)/avg_v);
}

double total_accel_cost(vector<vector<double>> traj, int target_vehicle, vector<double> delta, double T, map<int, Vehicle> predictions){
	//
	//
	//
	vector<double> s = traj[0];
	vector<double> d = traj[1];
	double t = traj[2][0];

	vector<double> s_dot = differentiate(s);
    vector<double> s_d_dot = differentiate(s_dot);
    double total_acc = 0.0;
 	double dt = T/100.0;
    for (int i=0; i<100;i++){
        t = dt * i;
        double acc = f(s_d_dot, t);
        total_acc += abs(acc*dt);
    }
    double acc_per_second = total_acc / T;

    return logistic(acc_per_second / EXPECTED_ACC_IN_ONE_SEC );

}

double max_accel_cost(vector<vector<double>> traj, int target_vehicle, vector<double> delta, double T, map<int, Vehicle> predictions){
	//
	//
	//
	vector<double> s = traj[0];
	vector<double> d = traj[1];
	double t = traj[2][0];

	vector<double> s_dot = differentiate(s);
    vector<double> s_d_dot = differentiate(s_dot);
    double max_acc = 0.0;
 	double dt = T/100.0;
    for (int i=0; i<100;i++){
        t = dt * i;
        double acc = abs(f(s_d_dot, t));
        if(acc >= max_acc) {
        	max_acc = acc;
        }
    }
    if (max_acc > MAX_ACCEL) return 1;
    else return 0.0;
}


double max_jerk_cost(vector<vector<double>> traj, int target_vehicle, vector<double> delta, double T, map<int, Vehicle> predictions){
	//
	//
	//
	vector<double> s = traj[0];
	vector<double> d = traj[1];
	double t = traj[2][0];

	vector<double> s_dot = differentiate(s);
    vector<double> s_d_dot = differentiate(s_dot);
    vector<double> jerk_coefs = differentiate(s_d_dot);

    double max_jerk = 0.0;
 	double dt = T/100.0;
    for (int i=0; i<100;i++){
        t = dt * i;
        double jerk = abs(f(jerk_coefs, t));
        if(jerk >= max_jerk) {
        	max_jerk = jerk;
        }
    }
    if (max_jerk > MAX_JERK) return 1.0;
    else return 0.0;
}


double total_jerk_cost(vector<vector<double>> traj, int target_vehicle, vector<double> delta, double T, map<int, Vehicle> predictions){
	//
	//
	//
	vector<double> s = traj[0];
	vector<double> d = traj[1];
	double t = traj[2][0];

	vector<double> s_dot = differentiate(s);
    vector<double> s_d_dot = differentiate(s_dot);
    vector<double> jerk_coefs = differentiate(s_d_dot);

    double total_jerk = 0.0;
 	double dt = T/100.0;
    for (int i=0; i<100;i++){
        t = dt * i;
        double jerk = abs(f(jerk_coefs, t));

        total_jerk += abs(jerk*dt)
    }
    double jerk_per_second = total_jerk / T;
    return logistic(jerk_per_second / EXPECTED_JERK_IN_ONE_SEC );
}
