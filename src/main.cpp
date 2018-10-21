#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "constants.h"
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "helpers.h"
#include "cost_functions.h"
#include "ptg.h"
#include "road.h"
#include <time.h>



using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// for convenience
using json = nlohmann::json;
static map<int, Vehicle> vehicles;
static float dt;
static Vehicle ego;
static Vehicle newvf = Vehicle(0);
static int lane =1;
// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}
void printego(){
		Vehicle* v;
		cout << "\t\t\t\t"   << setw(12) << "id\t" << setw(12) << "lane\t" << setw(12) << "s\t" << setw(12) << "d\t" << setw(12) << "v\t"<< setw(12)
			 << "x\t"    << setw(12) << "y\t" << setw(12) << "a\t"   << setw(12)
			 << "yaw\t"<< setw(12) << "State" << endl;

		v= &ego;
		std::cout << "\t\t\t\t" << setw(12) << "EGO"<< "\t" << setw(12) << v->lane << "\t" << setw(12) << v->s   << "\t" << setw(12) << v->d
				  << "\t" << setw(12) << v->v*2.24 << "\t" << setw(12) << v->x
				  << "\t" << setw(12) << v->y << "\t" << setw(12) << v->a    << "\t" << setw(12) << rad2deg(v->yaw) << "\t" << setw(12) << v->state << endl;

}
void printvehicles() {
	Vehicle* v;
	for (int i=0;i<vehicles.size();i++){
		v= &vehicles[i];
		if (true) {// v->in_range==true){
			cout <<"i " << i;
			std::cout << "\t" << "egodist " << setw(12) << v->dist_to_ego << "\t" << "in range " <<  v->in_range  << setw(12) << i    << "\t" << setw(12) << v->lane << "\t" << setw(12) << v->s   << "\t" << setw(12) << v->d
					  << "\t" << setw(12) << v->v*2.24 << "\t" << setw(12) << v->x
					  << "\t" << setw(12) << v->y << "\t" << setw(12) << v->a    << "\t" << setw(12) << rad2deg(v->yaw) << "\t" << setw(12) << v->state << endl;
		}
	}
	cout<<endl;

}

void printpredictions(const map<int, vector<Vehicle>>& predictions) {
	for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it) {
		int key = it->first;
		vector<Vehicle> path = it->second;
		for (int j=0;j<path.size();j++){
			Vehicle* v = &path[j];
			if (true) {// v->in_range==true){
				cout <<"j " << j;
				std::cout << "\t" << "egodist " << v->dist_to_ego << "\t" << "id " <<  v->id  << setw(12) << v->id    << "\t" << setw(12) << v->lane << "\t" << setw(12) << v->s   << "\t" << setw(12) << v->d
						  << "\t" << setw(12) << v->v*2.24 << "\t" << setw(12) << v->vx
						  << "\t" << setw(12) << v->vy << "\t" << setw(12) << v->a    << "\t" << setw(12) << rad2deg(v->yaw) << "\t" << setw(12) << v->state << endl;
			}
		}
	}
	cout<<endl;

}


void printit(int i) {
	Vehicle* v;
	cout << "\t"   << setw(12) << "id\t" << setw(12) << "lane\t" << setw(12) << "s\t" << setw(12) << "d\t" << setw(12) << "v\t"<< setw(12)
		 << "x\t"    << setw(12) << "y\t" << setw(12) << "a\t"   << setw(12)
		 << "State" << endl;
	cout <<"i " << i;
	v= &vehicles[i];
	std::cout << "\t" << setw(12) << i    << "\t" << setw(12) << v->lane << "\t" << setw(12) << v->s  << "\t" << setw(12) << v->d
			  << "\t" << setw(12) << v->v*2.24 << "\t" << setw(12) << v->x
			  << "\t" << setw(12) << v->y << "\t" << setw(12) << v->a    << "\t" << setw(12) << rad2deg(v->yaw) << "\t" << setw(12) << v->state << endl;

}

void update_vehicle(int id, float s, float d, float x, float y, float vx, float vy,float dt) {

	Vehicle* vehicle = &vehicles[id];
	vehicle->lane =  (d/4);
	vehicle->s = s;
	float newv= sqrt(vx*vx+vy*vy);// [m/s]
	vehicle->a = (newv-vehicle->v)/dt;// in [m/s2]
	vehicle->yaw = atan2(vy,vx);
	vehicle->speedUpdate(newv);// in [m/s]
	vehicle->d = d;
	vehicle->x = x;
	vehicle->y = y;
	vehicle->vx = vx;// in [m/s]
	vehicle->vy = vy;// in [m/s]
	int dot_sign = vx*(x-ego.x)+vy*(y-ego.y)<0.0? -1: 1;
	vehicle->dist_to_ego = distance(ego.x,ego.y,x,y)*dot_sign;
	vehicle->in_range = (vehicle->dist_to_ego <= in_range_dist && ((vehicle->dist_to_ego > -in_range_dist/4 && vehicle->v<ego.v) || (vehicle->dist_to_ego > -in_range_dist/3)));
	vehicle->dt = dt;

}

void update_ego(float s, float d, float x, float y, float yaw, float v,float dt) {//v is expected to be given  in [m/s]

	ego.s = s;
	float prev_v = ego.v;
	ego.speedUpdate(v);// in [m/s]
	ego.a = (prev_v-ego.v)/dt; // in [m/s2]
	ego.d = d;
	ego.x = x;
	ego.y = y;
	ego.vx = v*cos(yaw);// in [m/s]
	ego.vy = v*sin(yaw); // in [m/s]
	ego.yaw = yaw;
	ego.goal_s = s +1000;
	ego.lane =lane;
	ego.dt = dt;
}


vector<Vehicle> next_trajectory() {
	map<int ,vector<Vehicle> > predictions;

	map<int, Vehicle>::iterator it = vehicles.begin();
    while(it != vehicles.end())
    {	if(it->second.in_range){
			int v_id = it->first;
			vector<Vehicle> preds = it->second.generate_predictions(TRAJECTORYSTEPS);
			predictions[v_id] = preds;
		}
        it++;
    }

	vector<Vehicle> trajectory = ego.choose_next_state(predictions);
	ego.realize_next_state(trajectory);
	return trajectory;
}

int main() {
  uWS::Hub h;

  struct timespec start, finish;
  double elapsed;


  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
  	double x;
  	double y;
  	float s;
  	float d_x;
  	float d_y;
  	iss >> x;
  	iss >> y;
  	iss >> s;
  	iss >> d_x;
  	iss >> d_y;
  	map_waypoints_x.push_back(x);
  	map_waypoints_y.push_back(y);
  	map_waypoints_s.push_back(s);
  	map_waypoints_dx.push_back(d_x);
  	map_waypoints_dy.push_back(d_y);
  }
  double ref_vel = 49.88; //mph


  static int last_size =WAYPOINTSSIZE;

  vector<Vehicle> best_trajectory;
	static vector<double> next_x_vals1(WAYPOINTSSIZE);
	static vector<double> next_y_vals1(WAYPOINTSSIZE);

  clock_gettime(CLOCK_MONOTONIC, &start);

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&ref_vel, &best_trajectory, &start, &finish, &elapsed](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);
		// path points buffers
		//
		vector<double> next_x_vals;
		vector<double> next_y_vals;
      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        static bool done = false;
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data given to the Planner
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values 
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];
          	//cout << "end_path s " << end_path_s << " end_path_d " << end_path_d << endl;
          	//cout << "previous_path_x " << previous_path_x[0] << endl;
          	int prev_size = previous_path_x.size();
          //	cout << "prev size " << prev_size << endl;
          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	vector<vector<double>> sensor_fusion =  j[1]["sensor_fusion"];
			// Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m


          	/////////////////////////////////////////////////////////////////////////
          	//
          	// 	setup/update ego and traffic vehicles
          	//	run path trajectory planner state machine
          	//
          	if (!done){//add vehicles
          	    ego = Vehicle((car_d/4),EGO_ID,car_s,car_d,car_x,car_y,car_speed/2.24, deg2rad(car_yaw),"KL"); //#3
          	    ego.configure({ref_vel/2.24,3,10000,9.9});
          	    //printego();
          	    //traffic
          		for(int i=0;i<sensor_fusion.size();i++){
          			//Vehicle(int lane, float s, float d, float x, float y, float vx, float vy, string state){
          			//                               lane from d,                     vehic_s,           vehic_d,              vehic_x,          vehic_y,           vehic_vx,             vehic_vy,          state
           			Vehicle vehicle = Vehicle((int)(sensor_fusion[i][6]/4), sensor_fusion[i][5], sensor_fusion[i][6], sensor_fusion[i][1], sensor_fusion[i][2], sensor_fusion[i][3], sensor_fusion[i][4], "CS", i);//#2
          			vehicles.insert(std::pair<int,Vehicle>(i,vehicle));
           		}
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
#if DEBUG
              	cout << ">>>>>>>>>>>>>>>>>dt "<< dt << endl;
#endif
          		update_ego(car_s,car_d,car_x,car_y, deg2rad(car_yaw), car_speed*MPHTOMS, dt); //v is converted from MPH to [m/s]
#if DEBUG
          		printego();
#endif
          		for(int i=0;i<sensor_fusion.size();i++){
          			//void update_vehicle(int id, float s, float d, float x, float y, float vx in [m/s], float vy in [m/s],float dt) {
          			//            id, vehic_s,                vehic_d,                vehic_x,          vehic_y,           vehic_vxin [m/s],         vehic_vy in [m/s]
          			update_vehicle(i, sensor_fusion[i][5], sensor_fusion[i][6], sensor_fusion[i][1], sensor_fusion[i][2], sensor_fusion[i][3], sensor_fusion[i][4], dt);
          		}
#if DEBUG
          		printvehicles();
#endif
          		best_trajectory = next_trajectory();
#if DEBUG
          		print_trajectory(best_trajectory);
#endif
          		if (best_trajectory.size() !=0) {
          			lane = best_trajectory[1].lane;
          		}

          	    clock_gettime(CLOCK_MONOTONIC, &finish);

          	    elapsed = (finish.tv_sec - start.tv_sec);
          	    elapsed += (finish.tv_nsec - start.tv_nsec) / 1000000000.0;
#if DEBUG
          	    cout << "Elapsed time " << elapsed << endl;
#endif
          	}
          	//
          	// end trajectory planner FSM
          	//
          	///////////////////////////////////////////////////////////////////////


			// new s/d/v/a  from trajectory  planner FSM
			//
			double newsf;
			double newaf;
			double newlane;
			double newdf;
          	if(best_trajectory.size()!=0) {
    			Vehicle v = best_trajectory[1];
    			newsf = v.s;
    			newvf.speedUpdate(v.v);//ref_vel*MPHTOMS; //v.v;
    			newaf = v.a;
    			lane = v.lane;
    			newdf = (lane-1)*4+2;

          	} else { // this shouldn't occur in theory ... in the worst case it should return a keep lane trajectory
				newsf = car_s+60;
				newvf.speedUpdate(3.5); //car_speed/2.24;
				newvf.speedUpdate(3.5);
				newaf = 5;
				newdf = car_d;

          	}

			// Create a list of widely spaced (x,y) waypoints, evenly spaced at 30m

			vector<double> ptsx;
			vector<double> ptsy;
			// Reference x, y, yaw states
			double ref_x = car_x;
			double ref_y = car_y;
			double ref_yaw = deg2rad(car_yaw);

			// If previous size is almost empty, use the car as starting reference
			if (prev_size < 2) {
				// Use two points that make the path tangent to the car
				double prev_car_x = car_x - cos(car_yaw);
				double prev_car_y = car_y - sin(car_yaw);

				ptsx.push_back(prev_car_x);
				ptsx.push_back(car_x);

				ptsy.push_back(prev_car_y);
				ptsy.push_back(car_y);
			} else {
				// Use the previous path's endpoint as starting ref
				// Redefine reference state as previous path end point

				// Last point
				ref_x = previous_path_x[prev_size-1];
				ref_y = previous_path_y[prev_size-1];

				// 2nd-to-last point
				double ref_x_prev = previous_path_x[prev_size-2];
				double ref_y_prev = previous_path_y[prev_size-2];
				ref_yaw = atan2(ref_y-ref_y_prev, ref_x-ref_x_prev);

				// Use two points that make the path tangent to the path's previous endpoint
				ptsx.push_back(ref_x_prev);
				ptsx.push_back(ref_x);

				ptsy.push_back(ref_y_prev);
				ptsy.push_back(ref_y);
			}
			int temp = (2+4*(lane));
			vector<double> next_wp0 = getXY(car_s+60, (2+4*(lane)), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> next_wp1 = getXY(car_s+120, (2+4*(lane)), map_waypoints_s, map_waypoints_x, map_waypoints_y);
			vector<double> next_wp2 = getXY(car_s+180, (2+4*(lane)), map_waypoints_s, map_waypoints_x, map_waypoints_y);

			ptsx.push_back(next_wp0[0]);
			ptsx.push_back(next_wp1[0]);
			ptsx.push_back(next_wp2[0]);

			ptsy.push_back(next_wp0[1]);
			ptsy.push_back(next_wp1[1]);
			ptsy.push_back(next_wp2[1]);

			for (int i = 0; i < ptsx.size(); i++) {
				// Shift car reference angle to 0 degrees
				double shift_x = ptsx[i] - ref_x;
				double shift_y = ptsy[i] - ref_y;

				ptsx[i] = (shift_x * cos(0-ref_yaw) - shift_y * sin(0-ref_yaw));
				ptsy[i] = (shift_x * sin(0-ref_yaw) + shift_y * cos(0-ref_yaw));
			}

			// Create a spline called s
			tk::spline s;

			// Set (x,y) points to the spline
			s.set_points(ptsx, ptsy);

			// Define the actual (x,y) points we will use for the planner
			vector<double> next_x_vals;
			vector<double> next_y_vals;

			// Start with all the previous path points from last time
			for (int i = 0; i < previous_path_x.size(); i++) {
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
			}

			// Compute how to break up spline points so we travel at our desired reference velocity
			double target_x = 60.0;
			double target_y = s(target_x);
			double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
			double x_add_on = 0;

			// Fill up the rest of the path planner to always output 50 points
			for (int i = 1; i <= WAYPOINTSSIZE - previous_path_x.size(); i++) {
				double N = (target_dist/(WAYPOINTQUANT*newvf.v));
				double x_point = x_add_on + (target_x) / N;
				double y_point = s(x_point);

				x_add_on = x_point;

				double x_ref = x_point;
				double y_ref = y_point;

				// Rotate back to normal after rotating it earlier
				x_point = (x_ref * cos(ref_yaw) - y_ref*sin(ref_yaw));
				y_point = (x_ref * sin(ref_yaw) + y_ref*cos(ref_yaw));

				x_point += ref_x;
				y_point += ref_y;

				next_x_vals.push_back(x_point);
				next_y_vals.push_back(y_point);
			}
			json msgJson;
			msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          	//this_thread::sleep_for(chrono::milliseconds(1000));
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
