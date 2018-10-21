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
#include "helpers.h"
/**
 * Initializes Vehicle
 */

Vehicle::Vehicle(){}
Vehicle::Vehicle(float v){//#0 speed buffer
	this->mv_i = 0;
	this->v_buf = {v, v, v};
	// other object member are left undefined as they don't matter
}
Vehicle::Vehicle(int lane, float s, float d, float x, float y, float vx, float vy, string state, int id){ //#2
	this->mv_i = 0;
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
	this->v_buf = {this->v, this->v, this->v};
    this->a = 0;//max accel/2
    this->in_range = false;

} //traffic vehicles

Vehicle::Vehicle(int lane, int egoid,float s, float d, float x, float y, float v, float yaw,string state){//#3
	this->mv_i = 0;
	this->lane = (d/4)+1;
    this->s = s;
    this->d = d;
    this->x = x;
    this->y = y;
    this->v = v;
    this->v_buf = {v, v, v};
    this->vx= cos(yaw) * v;
    this->vy= sin(yaw) * v;
    this->yaw= yaw;
    this->state = state;
    max_acceleration = 10;
    this->a = 0;//max accel/2
    this->in_range = false;
    this->state = "KL";

} // ego


Vehicle::Vehicle(int lane, float s, float d, float v, float a, string state) {//#5
	this->mv_i = 0;
    this->lane = lane;
    this->s = s;
    this->v = v;
    this->v_buf = {v, v, v};
    this->a = a;
    this->state = state;
    max_acceleration = 10;
    this->d = d;
    this->in_range = false;
}

Vehicle::Vehicle(int lane, float s, float d, float v, float a, int id, string state) {//#6

    this->lane = lane;
    this->s = s;
    this->v = v;
    this->v_buf = {v, v, v};
    this->a = a;
    this->state = state;
    max_acceleration = 10;
    this->d = d;
    this->in_range = false;
    this->id = id;
}

Vehicle::Vehicle(int lane, float s, float v, float a, string state, string traj_type) {//#8
	this->mv_i = 0;
    this->lane = lane;
    this->s = s;
    this->v = v;
    this->v_buf = {v, v, v};
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

    float cost;
    vector<float> costs;
    vector<string> final_states;
    vector<vector<Vehicle>> final_trajectories;
#if DEBUG
    cout <<  ">DEBUG NEXT STATE" << endl;
#endif
    for (vector<string>::iterator it = states.begin(); it != states.end(); ++it) {
        vector<Vehicle> trajectory = generate_trajectory(*it, predictions);
        if (trajectory.size() != 0) {
#if DEBUG
        	cout << "\tState" << *it << "," << " L1 " << trajectory[0].lane << " L2 " << trajectory[1].lane << " " << endl;
#endif
            cost = calculate_cost(*this, predictions, trajectory);
            costs.push_back(cost);
            final_trajectories.push_back(trajectory);
        }
    }
#if DEBUG
    cout << endl;
#endif
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
#if DEBUG
    for (int i = 0;i<final_trajectories.size(); i++){
    	cout << "\tTrajectory Cost " << setw(12) << costs[sortedidx[i]] << " Trajectory Type  " << final_trajectories[sortedidx[i]][1].state <<  endl;
    }
    if (final_trajectories.size() !=3) {
    	cout << "\tTrajectory Cost " << setw(30) << " Trajectory Type  " <<  endl;
    	cout << "\tTrajectory Cost " << setw(30) << " Trajectory Type  " <<  endl;
    }
    cout << "#CHOOSE NEXT STATE " << endl;
#endif
    return final_trajectories[best_idx];
}

vector<string> Vehicle::successor_states() {
    /*
    Provides the possible next states given the current state for the FSM
    discussed in the course, with the exception that lane changes happen
    instantaneously, so LCL and LCR can only transition back to KL.
    */
#if DEBUG
	cout << ">FSM STATE BUFFER for Lane " << lane << endl;
#endif
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
        }
    } else if (state.compare("PLCR") == 0) {
        if (lane != 2) {
            states.push_back("PLCR");
            states.push_back("LCR");
        } else {
            states.push_back("PLCL");
        }
    }
#if DEBUG
    cout << "#FSM STATE BUFFER ";
    for (int i = 0;i<states.size();i++){
    	cout << states[i] << " ";
    }
    cout << endl;
#endif
    return states;
}


vector<Vehicle> Vehicle::generate_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Given a possible next state, generate the appropriate trajectory to realize the next state.
    */
#if DEBUG
	cout << "\t>GENERATE TRAJECTORY FOR FSM STATE " << state << endl;
#endif
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
    float dv = trajectory[1].v-trajectory[0].v;
    float ds = trajectory[1].s-trajectory[0].s;
    float time2trj = 60/trajectory[0].v;
#if DEBUG
	cout << "\t\tTrajectory Data  - End Speed " << setw(9) << trajectory[1].v << " Current Speed " <<  setw(9) << trajectory[0].v << " dv " << setw(9) << dv
						  << " ds " << setw(9) <<  ds << " a " << ds/dv << " time2trj " << setw(9) << time2trj << endl;
	if (abs(dv)/time2trj>10){
		//
		cout << "Trajectory END SPEED ERROR endspeed " << trajectory[1].v << endl;
				//trajectory[1].v = dv<0 ? -9.5*TIMESTEP + trajectory[0].v:9.5*TIMESTEP + trajectory[0].v;
	  	}
	cout << "\t\tnew speed " << trajectory[1].v;
    cout << "\t#GENERATE TRAJECTORY FOR FSM STATE  " << state << endl;
#endif
    return trajectory;
}

vector<float> Vehicle::get_kinematics(map<int, vector<Vehicle>> predictions, int lane) {
    /*
    Gets next timestep kinematics (position, velocity, acceleration)
    for a given lane. Tries to choose the maximum velocity and acceleration,
    given other vehicle positions and accel/velocity constraints.
    */
#if DEBUG
	cout << "\t\t\t>GET KINEMATICS " << endl;
#endif
	float max_velocity_accel_limit = this->max_acceleration*this->dt*0.95 + this->v;

    float new_position= 0.0;
    float new_velocity = 0.0;
    float new_accel = 0.0;
    float delta_v = 0.0;
	float delta_s = 0.0;
	float max_velocity_deccel_limit = 0.0;
	float updt_vel = 0.0;
    Vehicle vehicle_ahead;
    Vehicle vehicle_behind;
    float vec_v = 0.0;
    bool ahead=false;
    //float max_velocity_accel_limit = this->max_acceleration*time_to_tgt + this->v;
    if(get_vehicle_ahead(predictions, lane, vehicle_ahead)) {
    	ahead = true;
    	vec_v = vehicle_ahead.v*.95;
    	delta_v = (vec_v-this->v);
    	delta_s = (vehicle_ahead.s - this->s-KL_BUFFER);
    	if (delta_v < 0) { // if ego faster than vehicle ahead on same lane
			max_velocity_deccel_limit = this->v - this->max_acceleration*this->dt*0.95;
        	if (delta_s <= KL_BUFFER) {
        		new_accel = -this->max_acceleration*.95;
        		new_velocity = max(max_velocity_deccel_limit, vec_v);
        	} else { // to buffer
    			new_accel = delta_v*delta_v/delta_s;
    			updt_vel = this->v-new_accel*this->dt;
    			new_velocity = max(max(max_velocity_deccel_limit,updt_vel), vec_v);
        	}
    	} else { // if ego slower , meaning faster car pull in from another lane >> same case as if there was no vehicle.
    		new_velocity =   vehicle_ahead.v; //min(max_velocity_accel_limit, this->target_speed);
    		new_accel =  this->max_acceleration*0.95;
    	}
#if DEBUG
        cout << "\t\t\t\t   VehicAhead Speed choices DeltaV " << setw(9) << delta_v << "   DeltaS " << setw(9) << delta_s << " max decell "   << setw(9) << max_velocity_deccel_limit
        		<< "   Updt_vel " << setw(9) << updt_vel << "   new_acc " <<  setw(9) << new_accel  << "   max accel " << setw(9) << max_velocity_accel_limit
				<< "   tgt " << setw(9) << this->target_speed << "   ahead_v " << setw(9) << vec_v << "   this V " << setw(9) << this->v
				<< "   new_vel " << setw(9) << new_velocity << "   dt " << setw(9) << this->dt << endl;
#endif
    } else {
        new_velocity = min(max_velocity_accel_limit, this->target_speed);
#if DEBUG
        cout << "\t\t\t\t NoVehicAhead Speed choices DeltaV " << setw(9) << delta_v << "   DeltaS " << setw(9) << delta_s << "   max decell " << setw(9) << max_velocity_deccel_limit
        		<< "   Updt_vel " << setw(9) << updt_vel << "   new_acc " <<  setw(9) << new_accel  << "   max accel " << setw(9) << max_velocity_accel_limit
				<< "   tgt " << setw(9) << this->target_speed << "   ahead_v " << setw(9) << vec_v <<  "   this V " << setw(9) << this->v
				<< "   new_vel " << setw(9) << new_velocity << "   dt " << setw(9) << this->dt << endl;
#endif
        new_accel = this->max_acceleration*0.95;
    }
    new_position = this->s + new_velocity*TIMESTEP + (new_accel*TIMESTEP*TIMESTEP / 2.0);
#if DEBUG
    cout << "\t\t\t#GET KINEMATICS " << endl;
#endif
    return{new_position, new_velocity, new_accel};

}

vector<Vehicle> Vehicle::constant_speed_trajectory() {
    /*
    Generate a constant speed trajectory.
    */
#if DEBUG
	cout << "\t\t>CONSTANT SPEED TRAJECTORY" << endl;
#endif
    float next_pos = position_at(TIMESTEP);
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->d,this->v, 0, this->state), //#5
                                  Vehicle(this->lane, next_pos, this->v, 0, this->state,"constant_speed")};//#8
#if DEBUG
    cout << "\t\t#CONSTANT SPEED TRAJECTORY" << endl;
#endif
    return trajectory;
}


vector<Vehicle> Vehicle::keep_lane_trajectory(map<int, vector<Vehicle>> predictions) {
    /*
    Generate a keep lane trajectory.
    */
#if DEBUG
	cout << "\t\t>KEEP_LANE TRAJECTORY" << endl;
#endif
    vector<Vehicle> trajectory = {Vehicle(lane, this->s, this->d, this->v, this->a, state)}; //#5
    vector<float> kinematics = get_kinematics(predictions, this->lane);
    float new_s = kinematics[0];
    float new_v = kinematics[1];
    float new_a = kinematics[2];
    trajectory.push_back(Vehicle(this->lane, new_s,new_v,new_a, "KL", "keep_lane")); //#8
#if DEBUG
    print_trajectory(trajectory);
    cout << "\t\t#KEEP_LANE TRAJECTORY" << endl;
#endif
    return trajectory;
}


vector<Vehicle> Vehicle::prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Generate a trajectory preparing for a lane change.
    */
#if DEBUG
	cout << "\t\t>PREPARE_LANE TRAJECTORY" << endl;
#endif
    float new_s;
    float new_v;
    float new_a;
    Vehicle vehicle_behind;
    int new_lane = this->lane + lane_direction[state];
    vector<Vehicle> trajectory = {Vehicle(this->lane, this->s, this->d, this->v, this->a, this->state)};//#5
    vector<float> curr_lane_new_kinematics = get_kinematics(predictions, this->lane);
    if (get_vehicle_behind(predictions, this->lane, vehicle_behind)) {
        //Keep speed of current lane so as not to collide with car behind.
        new_s = curr_lane_new_kinematics[0];
        new_v = curr_lane_new_kinematics[1];
        new_a = curr_lane_new_kinematics[2];
#if DEBUG
        cout << "\t\t\tprepare car behind Vec " << vehicle_behind.id << endl;
#endif
    } else {
        vector<float> best_kinematics;
        vector<float> next_lane_new_kinematics = get_kinematics(predictions, new_lane);
        //Choose kinematics with lowest velocity.
        if (next_lane_new_kinematics[1] < curr_lane_new_kinematics[1]) {
            best_kinematics = next_lane_new_kinematics;
#if DEBUG
            cout << "\t\t\tnew lane kinematics " << endl;
#endif
        } else {
            best_kinematics = curr_lane_new_kinematics;
#if DEBUG
            cout << "\t\t\tcurr lane kinematics " << endl;
#endif
        }
        new_s = best_kinematics[0];
        new_v = best_kinematics[1];
        new_a = best_kinematics[2];
#if DEBUG
        cout << "\t\t\tprepare No car behind new s, v, a " << new_s << " " << new_v << " " << new_a << endl;
#endif
    }

    trajectory.push_back(Vehicle(this->lane, new_s,new_v, new_a, state, "PLC")); //#8
#if DEBUG
    print_trajectory(trajectory);
    cout << "\t\t#PREPARE_LANE TRAJECTORY" << endl;
#endif
    return trajectory;
}

vector<Vehicle> Vehicle::lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions) {
    /*
    Generate a lane change trajectory.
    */
#if DEBUG
	cout << "\t\t>CHANGE_LANE TRAJECTORY" << endl;
#endif
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
    trajectory.push_back(Vehicle(this->lane, this->s,this->d, this->v, this->a, this->state)); //#5
    vector<float> kinematics = get_kinematics(predictions, new_lane);
    trajectory.push_back(Vehicle(new_lane, kinematics[0],kinematics[1],kinematics[2], state, "lane_change")); //#8
#if DEBUG
    print_trajectory(trajectory);
    cout << "\t\t#CHANGE_LANE TRAJECTORY" << endl;
#endif
    return trajectory;
}



float Vehicle::position_at(float t) {
    return this->s + this->v*t ;
}

bool Vehicle::get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found behind the current vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
#if DEBUG
	cout << "\t\t\t>GET VEHICLE BEHIND" << endl;
#endif
    Vehicle temp_vehicle;
    bool found_vehicle = false;
    float dist = -15;
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
#if DEBUG
    if (found_vehicle)  cout << "\t\t\t\tBehind Vec " << temp_vehicle.id << " at dist " << temp_vehicle.dist_to_ego << endl;
    cout << "\t\t\t#GET VEHICLE BEHIND" << endl;
#endif
    return found_vehicle;
}

bool Vehicle::get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle) {
    /*
    Returns a true if a vehicle is found ahead of the ego vehicle, false otherwise. The passed reference
    rVehicle is updated if a vehicle is found.
    */
#if DEBUG
	cout << "\t\t\t\t>GET VEHICLE AHEAD" << endl;
#endif
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
#if DEBUG
    cout << "\t\t\t\t#GET VEHICLE AHEAD" << endl;
#endif
    return found_vehicle;
}

vector<Vehicle> Vehicle::generate_predictions(int horizon) {
    /*
    Generates predictions for non-ego vehicles to be used
    in trajectory generation for the ego vehicle.
    input is the number of timestep, which is TIMESTEP times  the horizon see constants.h */
	vector<Vehicle> predictions;
#if DEBUG
	cout << ">GENERATE PREDICTIONS" << endl;
	cout << "\tVec " << this->id << " lane " << this->lane << endl;
#endif
	for(int i = 0; i < horizon; i++) {
      float next_s = position_at(i*TIMESTEP/horizon);
      	  	  	  	  	  	  	  //int lane, float s, float v in [m/s], float a in [m/s2],  float dist_2_ego, float vx in [m/s], float vy in [m/s], int id, string stat
      predictions.push_back(Vehicle(this->lane, next_s, this->d, this->v, 0.0, this->id,"CS")); //round(predictions.back().v) // #6
#if DEBUG
      cout << "\ts " << setw(5) << round(predictions.back().s) << " v " << setw(5) << round(predictions.back().v/MPHTOMS) << endl;
#endif
  	}
    float ds = predictions[0].s-predictions[horizon-1].s;
#if DEBUG
	cout << "\t\tPrediction Data - Vehicle " << this->id << " Length " << setw(8) << ds << endl;
    cout << "#GENERATE PREDICTION" << endl;
#endif
    return predictions;

}


void Vehicle::realize_next_state(vector<Vehicle> trajectory) {
    /*
    Sets state and kinematics for ego vehicle using the last state of the trajectory.
    */
    Vehicle next_state = trajectory[1];
    this->state = next_state.state;
#if DEBUG
    cout << "Ego State " << next_state.state << endl;
#endif
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

void Vehicle::speedUpdate(float v){
	this->v_buf[this->mv_i] = v;
	this->v =  (this->v_buf[this->mv_i%3]+this->v_buf[(this->mv_i+1)%3]+this->v_buf[(this->mv_i+2)%3])/3;
	this->mv_i = (this->mv_i + 1) % 3;
}

