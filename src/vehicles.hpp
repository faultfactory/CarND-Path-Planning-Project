#include "math.h"
#include "helpers.hpp"
#include "json.hpp"
#include <deque>
#include <map>
#include "track.h"

#ifndef VEHICLES
#define VEHICLES

extern Track track;
extern double loop_time_ms;

class VehicleFrame
{
  
public:
  int id;   // sensor fusion has an ID. I'm just going to set it to -1 for the ego car.
  double x; // X position in m
  double y; // Y position in m
  double s; // Frenet Coordinate 
  double d; // Frenet Coorindate d
  double v_mag; // magnitude of velocity in m/s
  double yaw; // yaw direction
  int lane; //
  double dt; // milliseconds since last frame; 
  
  
  VehicleFrame(nlohmann::json j)
    {
      // Creates vehicle frame for Ego Vehicle
      // need to verify the output of json::parse() and adjust appropriately
      id = -1;
    	x = j[1]["x"];
    	y = j[1]["y"];
    	s = j[1]["s"];
    	d = j[1]["d"];
    	yaw = deg2rad(j[1]["yaw"]);
      v_mag = (j[1]["speed"]);
    	v_mag *= 0.44704;  // ego speed is delivered in mph but everything else is in meters, we will convert here and throw away mph
    	lane = getLane(d);
      dt = loop_time_ms; 
    };                // Ego constructor.
    
    VehicleFrame(std::vector<double> v)
    {
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
      dt = loop_time_ms;
    } //sensor fusion construct
};

class Vehicle
{
  // I am aware that this should be abstracted into a parent and two subclasses.
  // "God punishes those who optimze early"
  
  const int bufferMax = 10;
  const int estimationMin = 3;
  std::deque<VehicleFrame> buffer;
  public:
  Vehicle(VehicleFrame);
  
  Vehicle();
  void resetVehicle();
  // These parameters might be most useful for determining collisions;
  double s_rel; // Relative s coordinate; 
  double d_dot;
  double s_dot;
  double s_dot_dot;
  
  bool estimating; // value to tell external members to use or to ignore these state values
  bool updated; // tells Vehicle field whether this has been updated in this most recent frame
  
  VehicleFrame predictForward(double deltaT);
  VehicleFrame getMostRecentFrame() const;
  void addEgoFrame(nlohmann::json j  );
  void addFrame(VehicleFrame);
  void estimateValues();
};

class VehicleField
{
  Vehicle* ego_ptr;
 

  const double searchAhead=150.0;
  const double searchBehind=-150.0;
  const double max_s = 6945.554;

  std::map<int,Vehicle> localCars;
  public:
 VehicleField(Vehicle* ep){
    ego_ptr=ep;
  }
  void updateLocalCars(const VehicleFrame &egoNow,const std::vector<std::vector<double>> &incomingData);
  void checkLaneRightCurrent(const VehicleFrame &egoNow);
  void checkLaneLeftCurrent(const VehicleFrame &egoNow);
  int getFowardCar(int lane);
  double getFrenetTimeToCollision(int id);
  double getVehicleSpeed(int id);
  double getVehicleSDist(int id);
};



#endif