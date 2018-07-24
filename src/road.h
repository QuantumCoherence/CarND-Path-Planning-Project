#include <iostream>
#include <random>
#include <sstream>
#include <fstream>
#include <math.h>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <iterator>
#include "vehicle.h"

using namespace std;

class Road {
public:


  	string ego_rep = " *** ";

  	int ego_key = -1;

  	int num_lanes;

    vector<int> lane_speeds;

    int speed_limit;


    static map<int, Vehicle> vehicles;

    int vehicles_added = 0;


    /**
  	* Constructor
  	*/
  	Road(int speed_limit, vector<int> lane_speeds);

  	/**
  	* Destructor
  	*/
  	virtual ~Road();


  	void add_vehicle(int id, float s, float d, float x, float y, float vx, float vy, string state="CS");
  	void update_vehicle(int id, float s, float d, float x, float y, float vx, float vy,float dt);
  	void advance();
  	void printit(int id);
  	void printall();

  	void add_ego(int lane_num, int s, vector<int> config_data);

  	void cull();
};
