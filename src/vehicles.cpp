#include "vehicles.hpp"
#include "math.h"


VehicleFrame::VehicleFrame(json j)
{
  // Creates vehicle frame for Ego Vehicle
  // need to verify the output of json::parse() and adjust appropriately
  id = -1;
	x = j[1]["x"];
	y = j[1]["y"];
	s = j[1]["s"];
	d = j[1]["d"];
	yaw = j[1]["yaw"];
	v_mag = (j[1]["speed"])*2.24; // ego speed is delivered in mph but everything else is in meters, we will convert here and throw away mph
	lane = getLane(d);
}

VehicleFrame::VehicleFrame(vector<double> v){
  // Creates vehicle frame for other vehicle
  id = int(v[0]);
	
	x = j[1]["x"];
	y = j[1]["y"];

	double vx = v[3];
	double vy = v[4];
	
	v_mag = sqrt(vx*vx+vy*vy);
	yaw = atan2(vy,vx);
	
	s = v[5];
	d = v[6];
	lane = getLane(d);
}

Vehicle::Vehicle(VehicleFrame vf){
  buffer.push_back(vf);
  estimating = false;
}

void Vehicle::addFrame(VehicleFrame vf)
{
  buffer.push_back(vf);
  if(buffer.size()>estimationMin)
  {
    estimating = true;
  }
  if(buffer.size()>bufferMax)
  {
    buffer.pop();
  }
  if(estimating)
  {
    estimateValues();
  }
  
}

void Vehicle::estimateValues()
{
  
  std::vector<double> s_dot_list;
  std::vector<double> d_dot_list;
  std::vector<double> s_dot_dot_list;
  for(int i = 1; i<buffer.size(); i++)
  {
    s_dot_list.push_back(buffer[i].s-buffer[i-1].s);
    d_dot_list.push_back(buffer[i].d-buffer[i-1].d);
    if(i>1)
    {
      s_dot_dot_list.push_back(s_dot_list[i]-s_dot_list[i-1]);
    }
  }
  
  
}
