
          	//////////////////////////////////////////////////
          	//
          	//  sensor_fusion structure
          	//
          	//  [0]	car's unique ID,
          	//	[1]	car's x position in map coordinates,
          	//	[2]	car's y position in map coordinates,
          	//	[3]	car's x velocity in m/s,
          	//	[4]	car's y velocity in m/s,
          	//	[5]	car's s position in frenet coordinates,
          	//	[6]	car's d position in frenet coordinates.


          	/////////////////////////////////////////////////////////////////////////
          	//
          	// 	setup/update ego and traffic vehicles
          	//	run path trajectory planner state machine
          	//
          	if (!done){//add vehicles
          	    ego = Vehicle((car_d/4)+1,EGO_ID,car_s,car_d,car_x,car_y,car_speed/2.44, deg2rad(car_yaw),"KL");
          	    ego.configure({ref_vel/2.44,3,10000,10});
          	    //printego();
          	    //traffic
          		for(int i=0;i<sensor_fusion.size();i++){
          			//Vehicle(int lane, float s, float d, float x, float y, float vx, float vy, string state){
          			//                               lane from d,                     vehic_s,           vehic_d,              vehic_x,          vehic_y,           vehic_vx,             vehic_vy,          state
           			Vehicle vehicle = Vehicle((int)(sensor_fusion[i][6]/4)+1, sensor_fusion[i][5], sensor_fusion[i][6], sensor_fusion[i][1], sensor_fusion[i][2], sensor_fusion[i][3], sensor_fusion[i][4], "CS");
          			vehicles.insert(std::pair<int,Vehicle>(i,vehicle));
           		}
          		//printvehicles();
          		done = true;
          	} else {
          		// estimate cycle time of last loop
          		if (prev_size == WAYPOINTSSIZE) {
          			dt = 0.02;
          		} else if(prev_size ==last_size)  {
          			dt = 0.02*25;
          		}  else {
          			dt = 0.02 *(last_size-prev_size);
          		}
              	cout << ">>>>>>>>>>>>>>>>>dt "<< dt << endl;
          		update_ego(car_s,car_d,car_x,car_y, deg2rad(car_yaw), car_speed/2.44, dt);
          		printego();
          		for(int i=0;i<sensor_fusion.size();i++){
          			//void update_vehicle(int id, float s, float d, float x, float y, float vx, float vy,float dt) {
          			//            id, vehic_s,                vehic_d,                vehic_x,          vehic_y,           vehic_vx,             vehic_vy
          			update_vehicle(i, sensor_fusion[i][5], sensor_fusion[i][6], sensor_fusion[i][1], sensor_fusion[i][2], sensor_fusion[i][3], sensor_fusion[i][4], dt);
          		}
          		//printvehicles();
          		best_trajectory = next_trajectory();
          		print_trajectory(best_trajectory);
          	}
          	//
          	// end trajectory planner FSM
          	//
          	///////////////////////////////////////////////////////////////////////

          	json msgJson;

          	///////////////////////////////////////////////////////////////////////////
          	///////////////////////////////////////////////////////////////////////
          	////
          	////	simulator path points generation
          	////
          	////

          	////////////////////////////////////////////////////////////////////////
          	//
          	//	spline setup
          	//
          	vector<double> ptsx;
          	vector<double> ptsy;
          	double ref_x = car_x;
          	double ref_y = car_y;
			double ref_yaw = deg2rad(car_yaw);

			if (prev_size < 2){
				double prev_car_x = car_x - cos(car_yaw);
				double prev_car_y = car_y - sin(car_yaw);

			    ptsx.push_back(prev_car_x);
			    ptsx.push_back(car_x);

			    ptsy.push_back(prev_car_y);
			    ptsy.push_back(car_y);

			} else {
				ref_x = previous_path_x[prev_size-1];
				ref_y = previous_path_y[prev_size-1];

				double ref_x_prev = previous_path_x[prev_size-2];
				double ref_y_prev = previous_path_y[prev_size-2];
				ref_yaw = atan2(ref_y-ref_y_prev,ref_x-ref_x_prev);

				ptsx.push_back(ref_x_prev);
			    ptsx.push_back(ref_x);

			    ptsy.push_back(ref_y_prev);
			    ptsy.push_back(ref_y);
			}

			//in frenet add evenly 30m spaced ahead of the starting reference
			vector<double> next_wp0 = getXY(car_s+30,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
			vector<double> next_wp1 = getXY(car_s+60,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);
			vector<double> next_wp2 = getXY(car_s+90,(2+4*lane),map_waypoints_s,map_waypoints_x,map_waypoints_y);

			ptsx.push_back(next_wp0[0]);
			ptsx.push_back(next_wp1[0]);
			ptsx.push_back(next_wp2[0]);

			ptsy.push_back(next_wp0[1]);
			ptsy.push_back(next_wp1[1]);
			ptsy.push_back(next_wp2[1]);
			//cout << "ptsx\tptsy" << endl;
			//for (int i = 0; i < ptsx.size();i++){
			//			cout<< ptsx[i]<< "\t" << ptsy[i] << endl;
			//}
			//cout << "NEW ptsx\tptsy" << endl;
			for (int i = 0; i < ptsx.size();i++){
				//shift car angle ref to 0 degrees
				double shift_x = ptsx[i]-ref_x;
				double shift_y = ptsy[i]-ref_y;

				ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
				ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
				//cout<< ptsx[i]<< "\t" << ptsy[i] << endl;

			}

			tk::spline s;
			s.set_points(ptsx,ptsy);

			//
			//	spline ready
			////////////////////////////////////////////////////////////////////////////

          	////////////////////////////////////////////////////////////////////////////
			//
			//	path points
			//  here below spline and polynomial methods are implemented in parallel
			//	in a nuttshell, the ending point in frenet coords is used to estimate the
			//  simulator path points using spline and polynomial methods



			// new s/d/v/a  from trajectory  planner FSM
			//
			double newsf;
			double newvf;
			double newaf;
			double newlane;
			double newdf;
			double interval;
          	if(best_trajectory.size()!=0) {
    			Vehicle v = best_trajectory[1];
    			newsf = v.s;
    			newvf = v.v;
    			newaf = v.a;
    		//	newlane = v.lane;
    			newdf = car_d;//(newlane-1)*4+2;
    			interval =(newsf - car_s)/((newvf+(car_speed/2.44))/2);
    			cout << "interval " << interval << endl;

          	} else { // this shouldn't occur in theory ... in the worst case it should return a keep lane trajectory
				newsf = car_s+30;
				newvf = car_speed/2.44;
				newaf = 5;
				newdf = car_d;
				cout<<"no best trajectory"<<endl;
			    interval =(newsf - car_s)/((19.0+(car_speed/2.44))/2);
				cout << "interval " << interval << endl;

          	}
			//cout << "\tlane \t" << v.lane << "\td " << (v.lane-1)*4+2 << endl;
			///
			//	cout << "\t\t\tx\ty" << "\tprev size " <<  previous_path_x.size() << endl;
			//	cout << "last s and d\t " << end_path_s << "\t " << end_path_d << endl;
			//	for(int i=0; i < previous_path_x.size(); i++){
			//		next_x_vals.push_back(previous_path_x[i]);
			//		next_y_vals.push_back(previous_path_y[i]);
			//		cout<< "\t" << next_x_vals[i] << "\t" << next_y_vals[i] << endl;
			//	}
			//	cout << "car s & d\t " << car_s << "\t " << car_d << endl;

			////////////////////////////////////////
			//  polynomial trajectory generation
			//  problem is tuning it in a stable way which was poorly explained in the class
			//  Spline is a way to avoid the topic...

			vector<double> s_i = {car_s, car_speed/2.24,0};
			vector<double> s_f = {newsf,newvf,newaf};
			vector<double> scoeffs = JMT(s_i,s_f,interval);

			vector<double> d_i = {car_d, 0,0};
			vector<double> d_f = {newdf,0,0};
			vector<double> dcoeffs = JMT(d_i,d_f,interval);

			double dist_inc = 0.35;
			double dist_inc2 = newvf*WAYPOINTQUANT;

			///////////////////////////////////////
			// spline...
			//calculate how to break up spline points so that we travelat our desired ref velocity
			double target_x = 30.0;
			double target_y = s(target_x);
			double target_dist= sqrt((target_x)*(target_x)+(target_y)*(target_y));

			double x_add_on = 0;

			cout<< "\tspline\t\t\t\\t\t\t\tpolynomial"<<endl;
			cout<< "\ts\td\tx\ty\t\t\ts\td\tx\ty"<<endl;
			//buffers non used points from previous trajectory
			cout << "previous " << endl;
			for(int i =0; i < prev_size;i++){
				// spline points buffers
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
				cout << "\t" << previous_path_x[i] << setw(10) << "\t" 	<< setw(18) << previous_path_y[i];

				//polynomial trajectory points
				next_x_vals1[i] = next_x_vals1[i+(WAYPOINTSSIZE -prev_size)];
				next_y_vals1[i] = next_y_vals1[i+(WAYPOINTSSIZE -prev_size)];
				cout << setw(15) << next_x_vals1[i] <<  setw(15) <<  next_y_vals1[i] << endl;
			}
			cout << "new " << endl;
			// add the new points to the trajectory
			for(int i =1; i <= WAYPOINTSSIZE -prev_size;i++) {

				double N = (target_dist/(0.02*ref_vel));
				double x_point = x_add_on + (target_x/N);
				double y_point = s(x_point);

				x_add_on = x_point;

				double x_ref = x_point;
				double y_ref = y_point;

				x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
				y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

				x_point += ref_x;
				y_point += ref_y;

				next_x_vals.push_back(x_point);
				next_y_vals.push_back(y_point);
				cout << "\t" << setw(15) << x_point << setw(15) << y_point;
				//double next_s = car_s+(i+1)*dist_inc2;
				//double next_d = car_d+(i+1)*(newdf-car_d)/10;
				//cout << "\t" << next_s << "\t" << next_d;
				//vector<double> xy2 = getXY(next_s,next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
				//cout << "\t" << xy2[0] << "\t" << xy2[1];
				//next_x_vals1.push_back(xy2[0]);
				//next_y_vals1.push_back(xy2[1]);
				double next_s = f(scoeffs,(prev_size+i)*WAYPOINTQUANT);
				double next_d = f(dcoeffs,(prev_size+i)*WAYPOINTQUANT);
				cout << "\t" << setw(15) << next_s << setw(15) << next_d;
				vector<double> next_wp0 = getXY(next_s,next_d,map_waypoints_s,map_waypoints_x,map_waypoints_y);

				cout << setw(15) << next_wp0[0] <<  setw(15) << next_wp0[1] << endl;
				next_x_vals1[prev_size+i] = next_wp0[0];
				next_y_vals1[prev_size+i] = next_wp0[1];

				/*next_s = car_s+(i+1)*dist_inc;
				next_d = 14;
				cout << "\t\t\t" << next_s << "\t" << next_d;
				vector<double> xy = getXY(next_s,next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
				cout << "\t" << xy[0] << "\t" << xy[1] << endl;
				next_x_vals2.push_back(xy[0]);
				next_y_vals2.push_back(xy[1]);*/

			}
			last_size =next_x_vals.size();

			/*

			/////////////////////////////////////////////////////////////////////////////////
			// spline...
			//calculate how to break up spline points so that we travelat our desired ref velocity
			double target_x = 30.0;
			double target_y = s(target_x);
			double target_dist= sqrt((target_x)*(target_x)+(target_y)*(target_y));

			double x_add_on = 0;
			cout << "\tx\ty" << endl;
			for (int i =0; i <= 50-previous_path_x.size();i++){
				double N = (target_dist/(0.02*ref_vel/2.24));
				double x_point = x_add_on + (target_x/N);
				double y_point = s(x_point);

				x_add_on = x_point;

				double x_ref = x_point;
				double y_ref = y_point;

				x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
				y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

				x_point += ref_x;
				y_point += ref_y;

				next_x_vals.push_back(x_point);
				next_y_vals.push_back(y_point);
				cout<< "\t" << x_point << "\t" << y_point << endl;


			}
			cout << endl;

	 	 	    */





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
