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

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;

// for convenience
using json = nlohmann::json;
static map<int, Vehicle> vehicles;
static float dt;
static Vehicle ego;
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
		cout << "\t"   << setw(12) << "id\t" << setw(12) << "lane\t" << setw(12) << "s\t" << setw(12) << "d\t" << setw(12) << "v\t"<< setw(12)
			 <<	"vx\t" << setw(12) << "vy\t" << setw(12) << "x\t"    << setw(12) << "y\t" << setw(12) << "a\t"   << setw(12)
			 << "yaw\t"<< setw(12) << "State" << endl;

		v= &vehicles[EGO_ID];
		std::cout << "\t" << setw(12) << "EGO"<< "\t" << setw(12) << v->lane << "\t" << setw(12) << v->s   << "\t" << setw(12) << v->d
				  << "\t" << setw(12) << v->v << "\t" << setw(12) << v->vx   << "\t" << setw(12) << v->vy  << "\t" << setw(12) << v->x
				  << "\t" << setw(12) << v->y << "\t" << setw(12) << v->a    << "\t" << setw(12) << rad2deg(v->yaw) << "\t" << setw(12) << v->state << endl;

}
void printvehicles() {
	Vehicle* v;
	for (int i=0;i<vehicles.size();i++){
		cout <<"i " << i;
		v= &vehicles[i];
		std::cout << "\t" << setw(12) << i    << "\t" << setw(12) << v->lane << "\t" << setw(12) << v->s   << "\t" << setw(12) << v->d
				  << "\t" << setw(12) << v->v << "\t" << setw(12) << v->vx   << "\t" << setw(12) << v->vy  << "\t" << setw(12) << v->x
				  << "\t" << setw(12) << v->y << "\t" << setw(12) << v->a    << "\t" << setw(12) << rad2deg(v->yaw) << "\t" << setw(12) << v->state << endl;
	}
	cout<<endl;

}
void printit(int i) {
	Vehicle* v;
	cout << "\t"   << setw(12) << "id\t" << setw(12) << "lane\t" << setw(12) << "s\t" << setw(12) << "d\t" << setw(12) << "v\t"<< setw(12)
		 <<	"vx\t" << setw(12) << "vy\t" << setw(12) << "x\t"    << setw(12) << "y\t" << setw(12) << "a\t"   << setw(12)
		 << "State" << endl;
	cout <<"i " << i;
	v= &vehicles[i];
	std::cout << "\t" << setw(12) << i    << "\t" << setw(12) << v->lane << "\t" << setw(12) << v->s  << "\t" << setw(12) << v->d
			  << "\t" << setw(12) << v->v << "\t" << setw(12) << v->vx   << "\t" << setw(12) << v->vy  << "\t" << setw(12) << v->x
			  << "\t" << setw(12) << v->y << "\t" << setw(12) << v->a    << "\t" << setw(12) << rad2deg(v->yaw) << "\t" << setw(12) << v->state << endl;

}
void print_trajectory(vector<Vehicle> trajectory) {
	Vehicle* v;
	cout << "\t"   << setw(12) << "id\t" << setw(12) << "lane\t" << setw(12) << "s\t" << setw(12) << "d\t" << setw(12) << "v\t"<< setw(12)
		 <<	"vx\t" << setw(12) << "vy\t" << setw(12) << "x\t"    << setw(12) << "y\t" << setw(12) << "a\t"   << setw(12)
		 << "State" << endl;
	cout <<"Start ";
	v= &trajectory[0];
	std::cout << "\t" << setw(12) <<  0   << "\t" << setw(12) << v->lane << "\t" << setw(12) << v->s  << "\t" << setw(12) << v->d
			  << "\t" << setw(12) << v->v << "\t" << setw(12) << v->vx   << "\t" << setw(12) << v->vy  << "\t" << setw(12) << v->x
			  << "\t" << setw(12) << v->y << "\t" << setw(12) << v->a    << "\t" << setw(12) << rad2deg(v->yaw) << "\t" << setw(12) << v->state << endl;
	cout <<"End ";
	v= &trajectory[1];
	std::cout << "\t" << setw(12) << 1    << "\t" << setw(12) << v->lane << "\t" << setw(12) << v->s  << "\t" << setw(12) << v->d
			  << "\t" << setw(12) << v->v << "\t" << setw(12) << v->vx   << "\t" << setw(12) << v->vy  << "\t" << setw(12) << v->x
			  << "\t" << setw(12) << v->y << "\t" << setw(12) << v->a    << "\t" << setw(12) << rad2deg(v->yaw) << "\t" << setw(12) << v->state << endl;
	cout << endl;
}


void update_vehicle(int id, float s, float d, float x, float y, float vx, float vy,float dt) {

	Vehicle* vehicle = &vehicles[id];
	vehicle->lane =  (d/4)+1;
	vehicle->s = s;
	float newv= sqrt(vx*vx+vy*vy);
	vehicle->a = (newv-vehicle->v)/dt;
	vehicle->yaw = atan2(vy,vx);
	vehicle->v= newv;
	vehicle->d = d;
	vehicle->x = x;
	vehicle->y = y;
	vehicle->vx = vx;
	vehicle->vy = vy;
}

void update_ego(float s, float d, float x, float y, float yaw, float v,float dt) {

	Vehicle* vehicle = &vehicles[EGO_ID];
	vehicle->lane =  (d/4)+1;
	vehicle->s = s;
	vehicle->a = (v-vehicle->v)/dt;
	vehicle->v = v;
	vehicle->d = d;
	vehicle->x = x;
	vehicle->y = y;
	vehicle->vx = v*cos(yaw);
	vehicle->vy = v*sin(yaw);
	vehicle->yaw = yaw;
}

vector<Vehicle> next_trajectory(float dt) {

	map<int ,vector<Vehicle> > predictions;

	map<int, Vehicle>::iterator it = vehicles.begin();
    while(it != vehicles.end())
    {
        int v_id = it->first;
        vector<Vehicle> preds = it->second.generate_predictions(2);
        predictions[v_id] = preds;
        it++;
    }
	vector<Vehicle> trajectory = ego.choose_next_state(predictions);
	ego.realize_next_state(trajectory);
	return trajectory;
}

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

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
  int lane = 1;
  double ref_vel = 49.50; //mph

  static int last_size =50;

  vector<Vehicle> best_trajectory;


  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy,&lane,&ref_vel, &best_trajectory](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
//	vector<vector<double>> sensor_fusion;

    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

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
          	int prev_size = previous_path_x.size();
          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion =  j[1]["sensor_fusion"];
          	dt = 0.02;
          	if (!done){//add vehicles
          	    ego = Vehicle(EGO_ID,car_s,car_d,car_x,car_y,car_speed, deg2rad(car_yaw),"KL");
          	    ego.configure({ref_vel,3,10000.0,2,10});
          	    //printego();
          	    //traffic
          		for(int i=0;i<sensor_fusion.size();i++){
          			Vehicle vehicle = Vehicle(sensor_fusion[i][5], sensor_fusion[i][6], sensor_fusion[i][1], sensor_fusion[i][2], sensor_fusion[i][3], sensor_fusion[i][4], "CS");
          			vehicles.insert(std::pair<int,Vehicle>(i,vehicle));
           		}
          		//printvehicles();
          		done = true;
          	} else {

          		if (prev_size == WAYPOINTSSIZE) {
          			dt = 0.02;
          		} else if(prev_size ==last_size)  {
          			dt = 0.02*25;
          		}  else {
          			dt = 0.02 *(last_size-prev_size);
          		}
              	cout << ">>>>>>>>>>>>>>>>>dt "<< dt << endl;
          		update_ego(car_s,car_d,car_x,car_y, deg2rad(car_yaw), car_speed, dt);
          		printego();
          		for(int i=0;i<sensor_fusion.size();i++){
          			update_vehicle(i, sensor_fusion[i][5], sensor_fusion[i][6], sensor_fusion[i][1], sensor_fusion[i][2], sensor_fusion[i][3], sensor_fusion[i][4], dt);
          		}
          		printvehicles();
          		best_trajectory = next_trajectory(dt);
          		print_trajectory(best_trajectory);
          	}

          	json msgJson;


    /*      	vector<double> ptsx;
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
*/

////////////////////////////////////////////////////////////////////////////////////////////////
//  simple polynomial trajectory generation
//  problem is tuning it in a stable way which was poorly explained in the class
// Spline is a way to avoid the topic...
			double newsf;
			double newvf;
			double newaf;
			double newlane;
			double newdf;
          	if(best_trajectory.size()!=0) {
				Vehicle v = best_trajectory[1];
				newsf = v.s;
				newvf = v.v;
				newaf = v.a;
				newlane = v.lane;
				newdf = newlane*4+2;
          	} else {
				newsf = car_s+30;
				newvf = car_speed;
				newaf = 0;
				newdf = car_d;
				cout<<"no best trajectory"<<endl;
          	}
          	///
          	//planner points
			vector<double> next_x_vals;
			vector<double> next_y_vals;
		//	cout << "\t\t\tx\ty" << "\tprev size " <<  previous_path_x.size() << endl;
		//	cout << "last s and d\t " << end_path_s << "\t " << end_path_d << endl;
			for(int i=0; i < previous_path_x.size(); i++){
				next_x_vals.push_back(previous_path_x[i]);
				next_y_vals.push_back(previous_path_y[i]);
		//		cout<< "\t" << next_x_vals[i] << "\t" << next_y_vals[i] << endl;
			}
		//	cout << "car s & d\t " << car_s << "\t " << car_d << endl;
			vector<double> s_i = {car_s, car_speed/2.24,0};
			vector<double> s_f = {newsf,newvf,newaf};
			double interval =1;// 100/((car_speed+ref_vel)/4.48);
		//	cout << "intervall " << interval << endl;
			vector<double> s = JMT(s_i,s_f,interval);

			vector<double> d_i = {car_d, 0,0};
			vector<double> d_f = {newdf,0,0};
			vector<double> d = JMT(d_i,d_f,interval);
		//	cout<<"New points " << endl;
			for(int i =previous_path_x.size(); i <= WAYPOINTSSIZE;i++) {
				double next_s = f(s,i*0.02);
				double next_d = f(d,i*0.02);
				vector<double> next_wp0 = getXY(next_s,next_d,map_waypoints_s,map_waypoints_x,map_waypoints_y);
				next_x_vals.push_back(next_wp0[0]);
				next_y_vals.push_back(next_wp0[1]);
		//		cout<< "\t" << next_s << "\t" << next_d << "\t" << next_wp0[0] <<"\t" << next_wp0[1]<< endl;
			}
			last_size =next_x_vals.size();
			//cout<<endl;
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

////////////////////////////////////////////////////////////
 // simplelane follower

 /*

			//planner points
			vector<double> next_x_vals;
			vector<double> next_y_vals;

			double dist_inc = 0.5;
			cout<< "\ts\td\tx\ty"<<endl;
			for(int i = 0; i < 50; i++)
			{
				  if(i==previous_path_x.size()){
					  cout <<"\tnew points" <<endl;
				  }
				  double next_s = car_s+(i+1)*dist_inc;
				  double next_d = 6;
				  cout << "\t" << next_s << "\t" << next_d;
				  vector<double> xy = getXY(next_s,next_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
				  cout << "\t" << xy[0] << "\t" << xy[1] << endl;
				  next_x_vals.push_back(xy[0]);
				  next_y_vals.push_back(xy[1]);
			}
*/
 ///////////////////////////////////////////////////////////
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
