#include "vehicles.hpp"
#include <numeric>
#include "tic_toc.h"

Vehicle::Vehicle(){};
Vehicle::Vehicle(VehicleFrame vf){
  buffer.push_back(vf);
  estimating = false;
}

void Vehicle::addEgoFrame(nlohmann::json j)
{
  VehicleFrame egoFrame(j);
  addFrame(egoFrame);
}

void Vehicle::addFrame(VehicleFrame vf)
{
  buffer.push_back(vf);
  if(buffer.size() >= estimationMin)
  {
    estimating = true;
  }
  if(buffer.size()>bufferMax)
  {
    buffer.pop_front();
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
  
  // Some code referenced here:
  // https://stackoverflow.com/questions/7616511/
  //calculate-mean-and-standard-deviation-from-a-vector-of-samples-in-c-using-boos
  double s_dot_sum      = std::accumulate(s_dot_list.begin(), s_dot_list.end(), 0.0);
  double s_dot_dot_sum  = std::accumulate(s_dot_dot_list.begin(), s_dot_dot_list.end(), 0.0);
  double d_dot_sum      = std::accumulate(d_dot_list.begin(), d_dot_list.end(), 0.0);
  
  s_dot = s_dot_sum/s_dot_list.size();
  s_dot_dot = s_dot_dot_sum/s_dot_dot_list.size();
  d_dot = d_dot_sum/d_dot_list.size();
}

VehicleFrame Vehicle::getMostRecentFrame() const
{
    return buffer.back();
}


// provide an estimated state for the vehicle at time delta T from most recent frame.
VehicleFrame Vehicle::predictForward(double deltaT)
{
 // We will make some lazy assumptions with regard to velocity magnitude in this function as this data
 // is not stored and is used to check lane availability and adjacency.
 VehicleFrame frameOut = getMostRecentFrame();
 
 frameOut.s += s_dot*deltaT + s_dot_dot*deltaT*deltaT;
 frameOut.d += d_dot*deltaT;
 frameOut.lane = getLane(frameOut.d);
}

void VehicleField::resetUpdatedFlags()
{

 std::cout<<"reset ";
 tic();
 
  auto it = localCars.begin();
  while(it != localCars.end())
  {
    it->second.updated = false;
  }
  toc();
}

void VehicleField::updateLocalCars(const VehicleFrame &egoNow,const std::vector<std::vector<double>> &incomingData)
{
  double myS = egoNow.s;

  // make a temporary vehicle
  // if the ego car is near the end or beginning of the loop, we will add a correction to all units of s;
  bool tooLow = (myS-searchBehind)<0.0;
  bool tooHigh = (myS + searchAhead)>max_s;
  double sCorrect = (max_s/2.0)*(-1.0*tooHigh + 1.0*tooLow);
  myS+=sCorrect;
  double highLim = myS + searchAhead;
  double lowLim = myS - searchBehind;
  for(auto i = incomingData.begin(); i != incomingData.end();i++)
  {
    VehicleFrame tmpVehFrm(*i);
    bool neighbor = ((tmpVehFrm.s+sCorrect) > lowLim) && ((tmpVehFrm.s+sCorrect) < highLim);
    if(neighbor)
    {
      int id = tmpVehFrm.id;
      if(localCars.count(id)==0)
      {
        localCars.emplace(std::make_pair(id,Vehicle(tmpVehFrm)));
        std::cout<<"+ ";
      }
      else
      {
        localCars.at(id).addFrame(tmpVehFrm);
        std::cout<<"^ ";
      }
      localCars.at(id).updated=true;
    }
  }
 removeOutOfRangeCars();


}

void VehicleField::removeOutOfRangeCars()
{
  tic();
  //Cleaning out old vehicles. Referenced wisdom from C++ beasts at stack overflow
  // https://stackoverflow.com/questions/42819461/iterating-over-map-and-erasing-element/42820005#42820005
  std::cout<<"remove ";
  if(localCars.size()!=0)
  {
    auto iter = localCars.begin();
    while(iter != localCars.end())
    {
      if(iter->second.updated = false)
      {
        iter = localCars.erase(iter);
        std::cout<<"- ";
      }
      else
      {
        iter++;
      }
      std::cout<<". ";
    }
  toc();
  }
  std::cout<<std::endl;
}

void VehicleField::checkLaneRight(const VehicleFrame &egoNow)
{
  double my_s = egoNow.s;
  double my_lane = egoNow.lane;
  if(my_lane == 2)
  {
    std::cout<<"Lane Right Unavailable"<<std::endl;
  } 
   for(auto car_iter = localCars.begin(); car_iter != localCars.end(); car_iter++)
   {
   
    VehicleFrame tgtFrm = car_iter->second.getMostRecentFrame();
    if(tgtFrm.lane == my_lane+1)
    {
      if(tgtFrm.s<(my_s +2.0) && tgtFrm.s>(my_s-2.0))
      {
        std::cout<<"RIGHT LANE BLOCKED"<<std::endl;
      }
    }
   }
}

