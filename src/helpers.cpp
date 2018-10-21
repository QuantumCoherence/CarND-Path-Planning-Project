/*
 * helpers.cpp
 *
 *  Created on: Jul 10, 2018
 *      Author: paolo
 */
#include <fstream>
#include <math.h>
#include <iostream>
#include <iomanip>
#include <vector>
#include <map>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "vehicle.h"
#include "constants.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

void print_trajectory(vector<Vehicle> trajectory) {
	if (trajectory.size() !=0) {
		Vehicle* v;
		cout << "\t"   << setw(12) << "id\t" << setw(12) << "lane\t" << setw(12) << "s\t" << setw(12) << "d\t" << setw(12) << "v\t"<< setw(12)
			 <<	"vx\t" << setw(12) << "vy\t" << setw(12) << "x\t"    << setw(12) << "y\t" << setw(12) << "a\t"   << setw(12)
			 << "State" << endl;
		cout <<"Start ";
		v= &trajectory[0];
		std::cout << "\t" << setw(12) <<  0   << "\t" << setw(12) << v->lane << "\t" << setw(12) << v->s  << "\t" << setw(12) << v->d
				  << "\t" << setw(12) << v->v*2.24 << "\t" << setw(12) << v->vx*2.24   << "\t" << setw(12) << v->vy*2.24  << "\t" << setw(12) << v->x
				  << "\t" << setw(12) << v->y << "\t" << setw(12) << v->a    << "\t" << setw(12) << rad2deg(v->yaw) << "\t" << setw(12) << v->state << endl;
		cout <<"End ";
		v= &trajectory[1];
		std::cout << "\t" << setw(12) << 1    << "\t" << setw(12) << v->lane << "\t" << setw(12) << v->s  << "\t" << setw(12) << v->d
				  << "\t" << setw(12) << v->v*2.24 << "\t" << setw(12) << v->vx*2.24   << "\t" << setw(12) << v->vy*2.24  << "\t" << setw(12) << v->x
				  << "\t" << setw(12) << v->y << "\t" << setw(12) << v->a    << "\t" << setw(12) << rad2deg(v->yaw) << "\t" << setw(12) << v->state << endl;
		cout << endl;
	} else {
		cout << "Empty best trajectory " << endl;
	}
}



double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}

int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{

	int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

	double map_x = maps_x[closestWaypoint];
	double map_y = maps_y[closestWaypoint];

	double heading = atan2((map_y-y),(map_x-x));

	double angle = fabs(theta-heading);
  angle = min(2*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
  if (closestWaypoint == maps_x.size())
  {
    closestWaypoint = 0;
  }
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int next_wp = NextWaypoint(x,y, theta, maps_x,maps_y);

	int prev_wp;
	prev_wp = next_wp-1;
	if(next_wp == 0)
	{
		prev_wp  = maps_x.size()-1;
	}

	double n_x = maps_x[next_wp]-maps_x[prev_wp];
	double n_y = maps_y[next_wp]-maps_y[prev_wp];
	double x_x = x - maps_x[prev_wp];
	double x_y = y - maps_y[prev_wp];

	// find the projection of x onto n
	double proj_norm = (x_x*n_x+x_y*n_y)/(n_x*n_x+n_y*n_y);
	double proj_x = proj_norm*n_x;
	double proj_y = proj_norm*n_y;

	double frenet_d = distance(x_x,x_y,proj_x,proj_y);

	//see if d value is positive or negative by comparing it to a center point

	double center_x = 1000-maps_x[prev_wp];
	double center_y = 2000-maps_y[prev_wp];
	double centerToPos = distance(center_x,center_y,x_x,x_y);
	double centerToRef = distance(center_x,center_y,proj_x,proj_y);

	if(centerToPos <= centerToRef)
	{
		frenet_d *= -1;
	}

	// calculate s value
	double frenet_s = 0;
	for(int i = 0; i < prev_wp; i++)
	{
		frenet_s += distance(maps_x[i],maps_y[i],maps_x[i+1],maps_y[i+1]);
	}

	frenet_s += distance(0,0,proj_x,proj_y);

	return {frenet_s,frenet_d};

}

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

}
double f(vector<double> c, double t){

	return c[0] + c[1] * t + c[2] * t*t + c[3] * t*t*t + c[4] * t*t*t*t + c[5] * t*t*t*t*t;

}

double logistic(double x){
    //
    //A function that returns a value between 0 and 1 for x in the
    //range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].
    //Useful for cost functions.
    //

    return 2.0 / (1 + exp(-x)) - 1.0;
}

vector<double> JMT(vector<double> start, vector<double> end, double T){
    /*
    Calculates Jerk Minimizing Trajectory for start, end and T.
    */

	MatrixXd A(3,3);
    VectorXd b(3);
	A(2,0) = 6*T;      A(1,0) = T*T;      A(2,1) = A(1,0)*12;
	A(0,0) = A(1,0)*T; A(1,1) = A(0,0)*4; A(2,2) = A(0,0)*20;
	A(0,1) = A(0,0)*T; A(1,2) = A(0,1)*5; A(0,2) = A(0,1)*T;
	A(1,0) = A(1,0)*3;

	b(0) = end[0] - start[0] - start[1]*T-0.5*start[2]*T*T;
	b(1) = end[1] - start[1] - start[2]*T;
	b(2) = end[2] - start[2];
	VectorXd x = A.colPivHouseholderQr().solve(b);
	//cout << start[0] << "," << start[1] << "," << 0.5*start[2] << "," <<  x <<endl;
	return {start[0],start[1],0.5*start[2],x(0),x(1),x(2)};

}

