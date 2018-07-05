#ifndef VEHICLES
#define VEHICLES

#include "math.h"
#include "helpers.hpp"
#include "json.hpp"
#include <deque>

class VehicleFrame 
{
  
public:
  int id;   // sensor fusion has an ID. I'm just going to set it to -1 for the ego car.
  double x; // X position in m
  double y; // Y position in m
  double s; // Frenet Coordinate S
  double d; // Frenet Coorindate d
  double v_mag; // magnitude of velocity in m/s
  double yaw; // yaw direction
  int lane; //
  
  
  VehicleFrame(nlohmann::json j)
    {
      // Creates vehicle frame for Ego Vehicle
      // need to verify the output of json::parse() and adjust appropriately
      id = -1;
    	x = j[1]["x"];
    	y = j[1]["y"];
    	s = j[1]["s"];
    	d = j[1]["d"];
    	yaw = j[1]["yaw"];
    	v_mag = (j[1]["speed"]); 
    	v_mag *= 2.24;  // ego speed is delivered in mph but everything else is in meters, we will convert here and throw away mph
    	lane = getLane(d);
    };                // Ego constructor.
    
    VehicleFrame(std::vector<double> v){
        // Creates vehicle frame for other vehicle
        id = int(v[0]);
    	
    	x = v[1];
    	y = v[2];
    
    	double vx = v[3];
    	double vy = v[4];
    	
    	v_mag = sqrt(vx*vx+vy*vy);
    	yaw = atan2(vy,vx);
    	
    	s = v[5];
    	d = v[6];
    	lane = getLane(d);
} //sensor fusion constructor
};

class Vehicle
{
  
  const int bufferMax = 5;
  const int estimationMin = 3;
  std::deque<VehicleFrame> buffer;
  public:
  Vehicle(VehicleFrame);
  
  //double world_yawrate;
  //double world_acc;
  
  // These parameters might be most useful for determining collisions;
  
  bool estimating; // value to tell external members to use or to ignore these state values
  double d_dot;
  double s_dot;
  double s_dot_dot;
  
  VehicleFrame predictForward(double deltaT);
  VehicleFrame getMostRecentFrame();
  void addFrame(VehicleFrame);
  void estimateValues();
};

#endif