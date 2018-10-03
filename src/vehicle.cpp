#include <algorithm>
#include <iomanip>
#include <iostream>
#include "vehicle.h"
#include <cmath>
#include <map>
#include <string>
#include <iterator>
#include "cost.h"
#include "constants.h"

/**
 * Initializes Vehicle
 */

Vehicle::Vehicle(){}
Vehicle::Vehicle(int lane, float s, float d, float x, float y, float v, float vx, float vy, float a, string state){
    this->lane = lane;
    this->s = s;
    this->d = d;
    this->x = x;
    this->y = y;
    this->v = v;
    this->vx= vx;
    this->vy= vy;
    this->a = a;
    this->state = state;
    max_acceleration = 10;
    this->in_range = false;

} // messed up

Vehicle::Vehicle(int lane, float s, float d, float x, float y, float vx, float vy, string state, int id){
	this->id = id;
	this->lane = lane;
    this->s = s;
    this->d = d;
    this->x = x;
    this->y = y;
    this->vx= vx;
    this->vy= vy;
    this->yaw = atan2(vy,vx);
    this->state = state;
    max_acceleration = 10;
    this->v=sqrt(vx*vx+vy*vy);
    this->a = 0;//max accel/2
    this->in_range = false;

} //traffic vehicles

Vehicle::Vehicle(int lane, int egoid,float s, float d, float x, float y, float v, float yaw,string state){

	this->lane = (d/4)+1;
    this->s = s;
    this->d = d;
    this->x = x;
    this->y = y;
    this->v = v;
    this->vx= cos(yaw) * v;
    this->vy= sin(yaw) * v;
    this->yaw= yaw;
    this->state = state;
    max_acceleration = 10;
    this->a = 0;//max accel/2
    this->in_range = false;
    this->state = "KL";

} // ego


Vehicle::Vehicle(int lane, float s, float v, float a, string state) {

    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    this->state = state;
    max_acceleration = 10;
    this->d = lane*4+2;
    this->in_range = false;
} // original
Vehicle::Vehicle(int lane, float s, float d, float v, float a, string state) {

    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    this->state = state;
    max_acceleration = 10;
    this->d = d;
    this->in_range = false;
}
Vehicle::Vehicle(int lane, float s, float v, float a,  float dist_2_ego, float x, float y, float vx, float vy, int id, string state) {

    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    this->state = state;
    max_acceleration = 10;
    this->d = lane*4+2;
    this->in_range = false;
    this->dist_to_ego = dist_2_ego;
    this->x = x;
    this->y = y;
    this->vx = vx;
    this->vy = vy;
    this->id = id;

} // ;predictions

Vehicle::Vehicle(int lane, float s, float v, float a, string state, string traj_type) {

    this->lane = lane;
    this->s = s;
    this->v = v;
    this->a = a;
    this->state = state;
    max_acceleration = 10;
    this->d = lane*4+2;
    this->in_range = false;
    this->traj_type = traj_type;
}

Vehicle::~Vehicle() {}


vector<Vehicle> Vehicle::choose_next_state(map<int, vector<Vehicle>> predictions) {
    /*
    Here you can implement the transition_function code from the Behavior Planning Pseudocode
    classroom concept. Your goal will be to return the best (lowest cost) trajectory corresponding
    to the next state.

    INPUT: A predictions map. This is a map of vehicle id keys with predicted
        vehicle trajectories as values. Trajectories are a vector of Vehicle objects representing
        the vehicle at the current timestep and one timestep in the future.
    OUTPUT: The the best (lowest cost) trajectory corresponding to the next ego vehicle state.

    */
    vector<string> states = successor_states();
    cout << "States Queue: ";
    for (int i=0;i<states.size();i++){
    	cout << states[i] << " >> ";
    }
    cout << endl;
    float cost;
    vector<float> costs;
    vector<string> final_states;
    vector<vector<Vehicle>> final_trajectories;

    for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
        vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
        if (trajectory.size() != 0) {
            cost = calculate_cost(*this, predictions, trajectory);
            costs.push_back(cost);
            final_trajectories.push_back(trajectory);
        }
    }
    vector<float>::iterator best_cost = min_element(begin(costs), end(costs));
    int best_idx = distance(begin(costs), best_cost);
    vector<int> sortedidx;
	sortedidx.push_back(0);
    for (int i=1; i<final_trajectories.size(); i++){
    	int j = 0;
    	for (vector<int>::iterator jt=sortedidx.begin(); jt != sortedidx.end();jt++){
    		if (costs[i] < costs[sortedidx[j]]){
    			sortedidx.insert(jt, i);
    			break;
    		} else {
    			if ((jt+1) == sortedidx.end()){
    				sortedidx.push_back(i);
    				break;
    			} else {
        			j++;
    			}
    		}
    	}
   	}
    if (states.size()== 1) {
    	for (int i=0;i<8+2*predictions.size()+2;i++){
    		cout << endl;
    	}
    }
    for (int i = 0;i<final_trajectories.size(); i++){
    	cout << "Trajectory Cost " << setw(12) << costs[sortedidx[i]] << " Trajectory Type  " << final_trajectories[sortedidx[i]][1].state <<  endl;
    }
    if (final_trajectories.size() !=3) {
    	cout << "Trajectory Cost " << setw(30) << " Trajectory Type  " <<  endl;
    	cout << "Trajectory Cost " << setw(30) << " Trajectory Type  " <<  endl;
    }
    return final_trajectories[best_idx];
}

vector<string> Vehicle::successor_states() {
    /*
    Provides the possible next states given the current state for the FSM
    discussed in the course, with the exception that lane changes happen
    instantaneously, so LCL and LCR can only transition back to KL.
    */
    vector<string> states;
    states.push_back("KL");
    string state = this->state;
    if(state.compare("KL") == 0) {
        states.push_back("PLCL");
        states.push_back("PLCR");
    } else if (state.compare("PLCL") == 0) {
        if (lane != 0) {
            states.push_back("PLCL");
            states.push_back("LCL");
        } else {
            states.push_back("PLCR");
            states.push_back("PLCL");
        }
    } else if (state.compare("PLCR") == 0) {
        if (lane != 2) {
            states.push_back("PLCR");
            states.push_back("LCR");
        } else {
            states.push_back("PLCL");
            states.push_back("PLCR");
        }
    }

    return states;
}


vector<Vehicle> Vehicle::generate_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
    vector<Vehicle> trajectory;
    if (state.compare("CS") == 0) {
        trajectory = constant_speed_trajectory();
    } else if (state.compare("KL") == 0) {
        trajectory = keep_lane_trajectory(predictions);
    } else if (state.compare("LCL") == 0 || state.compare("LCR") == 0) {
        trajectory = lane_change_trajectory(state, predictions);
    } else if (state.compare("PLCL") == 0 || state.compare("PLCR") == 0) {
        trajectory = prep_lane_change_trajectory(state, predictions);
    }
    cout << "state " << state << endl;
    return trajectory;
}

vector<float> Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane) {
    /*
    Gets next timestep kinematics (position, velocity, acceleration)
    for a given lane. Tries to choose the maximum velocity and acceleration,
    given other vehicle positions and accel/velocity constraints.
    */
    float new_position;
    float new_velocity;
    float new_accel;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;
    float dt = this->dt;
	float time_to_tgt =  (this->v + sqrt(this->v*this->v +80*this->max_acceleration))/(0.66*this->max_acceleration);
	float max_velocity_accel_limit = this->max_acceleration*time_to_tgt + this->v;
    if(get_vehicle_ahead(predictions, lane, vehicle_ahead)) {

		if (abs(vehicle_ahead.v - this->v) < 0.1) {
			new_accel = 0;
		} else {
			time_to_tgt = (vehicle_ahead.s - this->s - this-> preferred_buffer)/abs(vehicle_ahead.v - this->v);
			new_accel = (vehicle_ahead.v - this->v)/ time_to_tgt;
		}
		new_velocity = min(min(this->v+new_accel, max_velocity_accel_limit), this->target_speed);
    } else {
        new_velocity = min(max_velocity_accel_limit, this->target_speed);
        new_accel = (new_velocity - this->v)/time_to_tgt; //Equation: (v_1 - v_0)/t = acceleration
     }
    cout << endl;
	cout << "time_to_tgt  " << setw(9) << time_to_tgt << " new_accel "<< setw(9)  << new_accel << " new_velocity "<< setw(9)  << new_velocity << endl;

    new_position = this->s + new_velocity*time_to_tgt + new_accel*time_to_tgt*time_to_tgt / 2.0;

    if (new_velocity < 0) {
    	cout << " new velocity negative" << endl;
    }
    return{new_position, new_velocity, new_accel};

}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
    /*
    Generate a constant speed trajectory.
    */
    float next_pos = position_at(TIMESTEP);
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->d,this->v, this->a, this->state),
                                  Vehicle(this->lane, next_pos, this->v, 0, this->state,"constant_speed")};
    return trajectory;
}

vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> predictions) {
    /*
    Generate a keep lane trajectory.
    */
    vector<Vehicle> trajectory = {Vehicle(lane, this->s, this->d, this->v, this->a, state)};
    vector<float> kinematics = get_kinematics(predictions, this->lane);
    float new_s = kinematics[0];
    float new_v = kinematics[1];
    float new_a = kinematics[2];
    trajectory.push_back(Vehicle(this->lane, new_s,new_v,new_a, "KL", "keep_lane"));
    return trajectory;
}

vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Generate a trajectory preparing for a lane change.
    */
    float new_s;
    float new_v;
    float new_a;
    Vehicle vehicle_behind;
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->d, this->v, this->a, this->state)};
    vector<float> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);
    if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
        //Keep speed of current lane so as not to collide with car behind.
        new_s = curr_lane_new_kinematics[0];
        new_v = curr_lane_new_kinematics[1];
        new_a = curr_lane_new_kinematics[2];
        cout << " prepare car behind Vec " << vehicle_behind.id << endl;

    } else {
        vector<float> best_kinematics;
        vector<float> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
        //Choose kinematics with lowest velocity.
        if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
            best_kinematics = next_lane_new_kinematics;
        } else {
            best_kinematics = curr_lane_new_kinematics;
        }
        new_s = best_kinematics[0];
        new_v = best_kinematics[1];
        new_a = best_kinematics[2];
        cout << " prepare No car behind new s, v, a " << new_s << " " << new_v << " " << new_a << endl;
    }

    trajectory.push_back(Vehicle(this->lane, new_s,new_v, new_a, state, "PLC"));
    return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Generate a lane change trajectory.
    */
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory;
    Vehicle next_lane_vehicle;
    //Check if a lane change is possible (check if another vehicle occupies that spot).
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        next_lane_vehicle = it->second[0];
        if (next_lane_vehicle.s == this->s && next_lane_vehicle.lane == new_lane) {
            //If lane change is not possible, return empty trajectory.
            return trajectory;
        }
    }
    trajectory.push_back(Vehicle(this->lane, this->s,this->d, this->v, this->a, this->state));
    vector<float> kinematics = get_kinematics(predictions, new_lane);
    trajectory.push_back(Vehicle(new_lane, kinematics[0],kinematics[1],kinematics[2], state, "lane_change"));
    return trajectory;
}

float Vehicle::position_at(float t) {
    return this->s + this->v*t ;//+ this->a*t*t/2.0;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    Vehicle temp_vehicle;
    bool found_vehicle = false;
    float dist = -10;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); ++it) {
        temp_vehicle = it->second[0];
		if(temp_vehicle.lane == this->lane && temp_vehicle.dist_to_ego<0) {
			if(dist > temp_vehicle.dist_to_ego){
				dist = temp_vehicle.dist_to_ego;
				found_vehicle = true;
				rVehicle = temp_vehicle;
			}
		}
}
    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found ahead of the ego vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
    Vehicle temp_vehicle;
    bool found_vehicle = false;
    float dist = 99999;
    for (map<int, vector<Vehicle>>::iterator it = predictions.begin(); it != predictions.end(); it++) {
    	temp_vehicle = it->second[0];
        if (temp_vehicle.lane == this->lane && temp_vehicle.dist_to_ego>=0){
			if(dist > temp_vehicle.dist_to_ego){
				dist = temp_vehicle.dist_to_ego;
				found_vehicle = true;
				rVehicle = temp_vehicle;
			}
        }
    }
    return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon) {
    /*
    Generates predictions for non-ego vehicles to be used
    in trajectory generation for the ego vehicle.
    input is the number of timestep, which is TIMESTEP times  the horizon see constants.h */
	vector<Vehicle> predictions;
    for(int i = 0; i < horizon; i++) {
      float next_s = position_at(i*TIMESTEP/horizon);
      float next_v = 0;
      if (i < horizon-1) {
        next_v = (position_at((i+1)*TIMESTEP) - s)/TIMESTEP;
      }//int lane, float s, float v, float a,  float dist_2_ego, float vx, float vy, int id, string stat
      predictions.push_back(Vehicle(this->lane, next_s, next_v, 0.0, this->dist_to_ego, this->x, this->y, this->vx, this->vy, this->id,"CS"));
  	}
    return predictions;

}

void Vehicle::realize_next_state(vector<Vehicle> trajectory) {
    /*
    Sets state and kinematics for ego vehicle using the last state of the trajectory.
    */
    Vehicle next_state = trajectory[1];
    this->state = next_state.state;
    this->lane = next_state.lane;
    this->s = next_state.s;
    this->v = next_state.v;
    this->a = next_state.a;
}

void Vehicle::configure(vector<float> road_data) {
    /*
    Called by simulator before simulation begins. Sets various
    parameters which will impact the ego vehicle.
    */
    target_speed = road_data[0];
    lanes_available = road_data[1];
    goal_s = road_data[2];
    max_acceleration = road_data[3];
}



