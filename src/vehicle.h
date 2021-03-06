#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>
#include "constants.h"

using namespace std;

class Vehicle {
public:

  map<string, int> lane_direction = {{"PLCL", -1}, {"LCL", -1}, {"LCR", 1}, {"PLCR", 1}};


  struct collider{

    bool collision ; // is there a collision?
    int  time; // time collision happens

  };

  int L = 1;

  int preferred_buffer = KL_BUFFER; // impacts "keep lane" behavior.

  int lane;

  float s;
  float d;
  float x;
  float y;
  float v;
  int mv_i;
  vector<float> v_buf;
  float vx;
  float vy;
  float yaw;
  float a;
  string traj_type;
  float target_speed;

  int lanes_available;

  float max_acceleration;

  int id;

  int goal_s;
  float dt;
  bool in_range;
  string state;
  float dist_to_ego;

  /**
  * Constructor
  */
  Vehicle();
  Vehicle(float v);
  Vehicle(int lane, float s, float d, float x, float y, float vx, float vy, string state,  int id);//#2 1
  Vehicle(int lane, int egoid, float s, float d, float x, float y, float v, float yaw, string state="KL");//#3 1
  Vehicle(int lane, float s, float d, float v, float a, string state="CS"); //#5 4
  Vehicle(int lane, float s, float d, float v, float a, int id, string state="CS");//#6 1
  Vehicle(int lane, float s, float v, float a, string state, string traj_type);//#8 4


  /**
  * Destructor
  */
  virtual ~Vehicle();
  void update();
  vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> predictions);

  vector<string> successor_states();

  vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle>> predictions);

  vector<float> get_kinematics(map<int, vector<Vehicle>> predictions, int lane);

  vector<Vehicle> constant_speed_trajectory();

  vector<Vehicle> keep_lane_trajectory(map<int, vector<Vehicle>> predictions);

  vector<Vehicle> lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  vector<Vehicle> prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);


  float position_at(float t);

  bool get_vehicle_behind(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);

  bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);
  void speedUpdate(float v);

  vector<Vehicle> generate_predictions(int horizon=5);
  void realize_next_state(vector<Vehicle> trajectory);
    void configure(vector<float> road_data);

};

#endif
