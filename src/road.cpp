#include <iostream>
#include "road.h"
#include "vehicle.h"
#include <math.h>
#include <map>
#include <string>
#include <iterator>


/**
 * Initializes Road
 */
Road::Road(int speed_limit, vector<int> lane_speeds) {

    this->num_lanes = lane_speeds.size();
    this->lane_speeds = lane_speeds;
    this->speed_limit = speed_limit;
    }

Road::~Road() {}


void Road::add_vehicle(int id, float s, float d, float x, float y, float vx, float vy, string state) {

	static Vehicle vehicle = Vehicle(s, d, x, y, vx, vy, "CS");
	vehicles_added++;
	this->vehicles.insert(std::pair<int,Vehicle>(id,vehicle));
	this->printit(id);
}

void Road::printall() {
	Vehicle v;
	cout << "\t\tid\t\tlane\t\ts\t\td\t\tv\t\tvx\t\tvy\t\tx\t\ty\t\ta" << endl;
	for (int i=0;i<this->vehicles.size();i++){
			v= this->vehicles[i];
			std::cout << "\t\t" << i << "\t\t" << v.lane << "\t\t" << v.s <<"\t\t" << v.d <<"\t\t" << v.v <<"\t\t" << v.vx
					  << "\t\t" << v.vy   << "\t\t" << v.x <<"\t\t" << v.y <<"\t\t" << v.a<< endl;
	}
	cout<<endl;

}

void Road::printit(int id) {
	Vehicle v;
	cout << "\t\tid\t\tlane\t\ts\t\td\t\tv\t\tvx\t\tvy\t\tx\t\ty\t\ta" << endl;
	v= this->vehicles[id];
	std::cout << "\t\t" << id << "\t\t" << v.lane << "\t\t" << v.s <<"\t\t" << v.d <<"\t\t" << v.v <<"\t\t" << v.vx
			  << "\t\t" << v.vy   << "\t\t" << v.x <<"\t\t" << v.y <<"\t\t" << v.a<< endl;
	cout<<endl;

}

void Road::update_vehicle(int id, float s, float d, float x, float y, float vx, float vy,float dt) {

	Vehicle vehicle = this->vehicles[id];
	vehicle.lane =  (d/4)+1;
	vehicle.s = s;
	float newv= sqrt(vx*vx+vy+vy);
	vehicle.a = (newv-vehicle.v)/dt;
	vehicle.v= newv;
	vehicle.d = d;
	vehicle.x = x;
	vehicle.y = y;
	vehicle.vx = vx;
	vehicle.vy = vy;
	this->printit(id);
}

void Road::advance() {

	map<int ,vector<Vehicle> > predictions;

	map<int, Vehicle>::iterator it = this->vehicles.begin();
    while(it != this->vehicles.end())
    {
        int v_id = it->first;
        vector<Vehicle> preds = it->second.generate_predictions();
        predictions[v_id] = preds;
        it++;
    }
	it = this->vehicles.begin();
	while(it != this->vehicles.end())
    {
    	int v_id = it->first;
        if(v_id == ego_key)
        {
        	vector<Vehicle> trajectory = it->second.choose_next_state(predictions);
        	it->second.realize_next_state(trajectory);
        }
        else {
            it->second.increment(1);
        }
        it++;
    }

}


void Road::add_ego(int lane_num, int s, vector<int> config_data) {

	map<int, Vehicle>::iterator it = this->vehicles.begin();
    while(it != this->vehicles.end())
    {
    	int v_id = it->first;
        Vehicle v = it->second;
        if(v.lane == lane_num && v.s == s)
        {
        	this->vehicles.erase(v_id);
        }
        it++;
    }
    Vehicle ego = Vehicle(lane_num, s, 0,0,0,this->lane_speeds[lane_num],0,0, 0,"CS");
    ego.configure(config_data);
    ego.state = "KL";
    this->vehicles.insert(std::pair<int,Vehicle>(ego_key,ego));

}

