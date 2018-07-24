#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
public:

  map<string, int> lane_direction = {{"PLCL", 1}, {"LCL", 1}, {"LCR", -1}, {"PLCR", -1}};

  struct collider{

    bool collision ; // is there a collision?
    int  time; // time collision happens

  };

  int L = 1;

  int preferred_buffer = 6; // impacts "keep lane" behavior.

  int lane;

  float s;
  float d;
  float x;
  float y;
  float v;
  float vx;
  float vy;
  float yaw;
  float a;

  float target_speed;

  int lanes_available;

  float max_acceleration;

  int goal_lane;

  int goal_s;
  float dt;

  string state;

  /**
  * Constructor
  */
  Vehicle();
  Vehicle(int lane, float s, float d, float x, float y, float v, float vx, float vy, float a, string state="CS");
  Vehicle(float s, float d, float x, float y, float vx, float vy, string state="CS");
  Vehicle(int egoid, float s, float d, float x, float y, float v, float yaw, string state="KL");
  Vehicle(int lane, float s, float v, float a, string state="CS");


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

  vector<Vehicle> generate_predictions(int horizon=2);

  void realize_next_state(vector<Vehicle> trajectory);

  void configure(vector<float> road_data);

};

#endif
