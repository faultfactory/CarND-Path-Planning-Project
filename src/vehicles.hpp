#ifndef VEHICLES
#define VEHICLES

#include "helpers.hpp"
#include "json.hpp"

class VehicleFrame{
  
public:
  int id;   // sensor fusion has an ID. I'm just going to set it to -1 for the ego car.
  double x; // X position in m
  double y; // Y position in m
  double s; // Frenet Coordinate S
  double d; // Frenet Coorindate d
  double v_mag; // magnitude of velocity in m/s
  double yaw; // yaw direction
  int lane; //
  
  
  VehicleFrame(json);                // Ego constructor.
  VehicleFrame(std::vector<double>); //sensor fusion constructor
};

class Vehicle:
{
  
  const int bufferMax = 5;
  const int estimationMin = 3;
  std::vector<VehicleFrame> buffer;
  public:
  VehicleFrame(Vehicle);
  
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