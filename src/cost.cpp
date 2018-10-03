#include "cost.h"
#include "vehicle.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>
#include "constants.h"
#include "helpers.h"
#include <iomanip>
using namespace std;

float goal_distance_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data,  bool print) {
    /*
    Cost increases based on distance of intended lane (for planning a lane change) and final lane of trajectory.
    Cost of being out of goal lane also becomes larger as vehicle approaches goal distance.
    */
    float cost;
    float distance = data["distance_to_goal"];
    if (distance > 0) {
        cost = 1 - 2*exp(-(abs(2.0- data["intended_lane"] - data["final_lane"]) / distance));
    } else {
        cost = 1;
    }
    return cost;
}

float inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data, bool print) {
    /*
    Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than vehicle's target speed.
    */

    float proposed_speed_intended = lane_speed(predictions, data["intended_lane"]);
    if (proposed_speed_intended < 0) {
        proposed_speed_intended = vehicle.target_speed;
    }

    float proposed_speed_final = lane_speed(predictions, data["final_lane"]);
    if (proposed_speed_final < 0) {
        proposed_speed_final = vehicle.target_speed;
    }

    float cost = (2.0*vehicle.target_speed - proposed_speed_intended - proposed_speed_final)/vehicle.target_speed;
    return cost;
}

float lane_speed(const map<int, vector<Vehicle>> & predictions, int lane) {
    /*
    just find the ahead vehicle in a given lane and check the speed, Its speed is thee speed of the lane.
    */
    for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it) {
        int key = it->first;
        Vehicle vehicle = it->second[0];
        if (vehicle.lane == lane) {
            return vehicle.v;
        }
    }
    //Found no vehicle in the lane
    return -1.0;
}

float calculate_cost(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, const vector<Vehicle> & trajectory) {
    /*
    Sum weighted cost functions to get total cost for trajectory.
    */
    map<string, float> trajectory_data = get_helper_data(vehicle, trajectory, predictions);
    float cost = 0.0;

    //Add additional cost functions here.
    vector< function<float(const Vehicle & , const vector<Vehicle> &, const map<int, vector<Vehicle>> &, map<string, float> &, bool)>> cf_list = {goal_distance_cost, inefficiency_cost, collision_cost}; //, buffer_cost};
    vector<float> weight_list = {REACH_GOAL, EFFICIENCY, COLLISION};//, BUFFER};

    for (int i = 0; i < cf_list.size(); i++) {
    	bool print = (i==2);
    	float new_cost = weight_list[i]*cf_list[i](vehicle, trajectory, predictions, trajectory_data, print);
        cost += new_cost;
    }

    return cost;

}

map<string, float> get_helper_data(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions) {
    /*
    Generate helper data to use in cost functions:
    indended_lane: the current lane +/- 1 if vehicle is planning or executing a lane change.
    final_lane: the lane of the vehicle at the end of the trajectory.
    distance_to_goal: the distance of the vehicle to the goal.

    Note that indended_lane and final_lane are both included to help differentiate between planning and executing
    a lane change in the cost functions.
    */
    map<string, float> trajectory_data;
    Vehicle trajectory_last = trajectory[1];
    float intended_lane;

    if (trajectory_last.state.compare("PLCL") == 0) {
        intended_lane = trajectory_last.lane -1;
    } else if (trajectory_last.state.compare("PLCR") == 0) {
        intended_lane = trajectory_last.lane + 1;
    } else {
        intended_lane = trajectory_last.lane;
    }

    float distance_to_goal = vehicle.goal_s - trajectory_last.s;
    float final_lane = trajectory_last.lane;
    trajectory_data["intended_lane"] = intended_lane;
    trajectory_data["final_lane"] = final_lane;
    trajectory_data["distance_to_goal"] = distance_to_goal;
    return trajectory_data;
}

float collision_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, float> & data, bool print) {
    //"""
    //Binary cost function which penalizes collisions.
    //"""

	return  in_range_dist/nearest_approach_to_any_vehicle(vehicle, trajectory, predictions, print);


}



/*
float buffer_cost(vector<vector<float>> traj, int target_vehicle, vector<float> delta, float T, map<int, Vehicle> predictions){
    //"""
    //Penalizes getting close to other vehicles.
    //"""

	double nearest = nearest_approach_to_any_vehicle(traj, predictions);
    return logistic(2*VEHICLE_RADIUS / nearest);
}


*/

float nearest_approach_to_any_vehicle(const Vehicle& ego,  const vector<Vehicle>& trajectory,const  map<int,vector<Vehicle>>& predictions, bool print){
    //
    //Calculates the closest distance to any vehicle during a trajectory.
    //

    float closest = 999999;
    for (auto const& v : predictions){
        float d = nearest_approach(ego, trajectory,v.second, print);
        if (d < closest){
            closest = d;
        }
    }
    if (print) cout << "overall " << setw(12) << closest << endl;
    return closest < 0.5 ? 0.5 : closest;
}

float nearest_approach(const Vehicle& ego, const vector<Vehicle>& trajectory, const vector<Vehicle>& vehicle, bool print){
    float closest = 999999;
    if (print) cout << "Vec " << setw(8) << vehicle[0].id;
    float new_s = trajectory[1].s;
    float new_d = trajectory[1].d;
    int steps =  vehicle.size();
    float delta_s = (new_s-ego.s)/steps;
    float delta_d = (new_d-ego.d)/steps;
    for (int i=0;i<steps;i++){
        float dist = distance(ego.s+i*delta_s,ego.d+i*delta_d, vehicle[i].s, vehicle[i].d);
        if (dist < closest){
            closest = dist;
        }
        if (print) cout << setw(8) << " d " << dist;
    }
    if (print) cout << " closest " << setw(9) << closest << endl;
    return closest;
}
