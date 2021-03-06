#include "vehicles.hpp"
#include <numeric>


Vehicle::Vehicle(){};
Vehicle::Vehicle(VehicleFrame vf)
{
  buffer.push_back(vf);
  estimating = false;
}

void EgoVehicle::addEgoFrame(EgoFrame egoFrm)
{
  addFrame(egoFrm);
  previousPath = egoFrm.pathStatus;

}

void ExternalVehicle::addExternalFrame(VehicleFrame vf)
{
  addFrame(vf); 

}

void Vehicle::addFrame(VehicleFrame vf)
{
  buffer.push_back(vf);
  if (buffer.size() >= estimationMin)
  {
    estimating = true;
  }
  if (buffer.size() > bufferMax)
  {
    buffer.pop_front();
  }
  if (estimating)
  {
    estimateValues();
  }
}

void Vehicle::estimateValues()
{

  std::vector<double> s_dot_list;
  std::vector<double> d_dot_list;
  std::vector<double> d_dot_dot_list;
  std::vector<double> s_dot_dot_list;
  for (int i = 1; i < buffer.size(); i++)
  {
    double dt = buffer.at(i).dt; 
    s_dot_list.push_back((buffer.at(i).s - buffer.at(i-1).s)/dt);
    d_dot_list.push_back((buffer.at(i).d - buffer.at(i-1).d)/dt);
    if (i > 2)
    {
      s_dot_dot_list.push_back((s_dot_list.at(i-1)- s_dot_list.at(i-2))/dt);
      d_dot_dot_list.push_back((d_dot_list.at(i-1) - d_dot_list.at(i-2))/dt);
    }
  }

  // Some code referenced here:
  // https://stackoverflow.com/questions/7616511/
  //calculate-mean-and-standard-deviation-from-a-vector-of-samples-in-c-using-boos
  double s_dot_sum = std::accumulate(s_dot_list.begin(), s_dot_list.end(), 0.0);
  double s_dot_dot_sum = std::accumulate(s_dot_dot_list.begin(), s_dot_dot_list.end(), 0.0);
  double d_dot_sum = std::accumulate(d_dot_list.begin(), d_dot_list.end(), 0.0);
  double d_dot_dot_sum = std::accumulate(d_dot_dot_list.begin(), d_dot_dot_list.end(), 0.0);

  s_dot = s_dot_sum / double(s_dot_list.size());
  s_dot_dot = s_dot_dot_sum / double(s_dot_dot_list.size());
  d_dot = d_dot_sum / double(d_dot_list.size());
  d_dot_dot = d_dot_dot_sum / double(d_dot_dot_list.size());
}

VehicleFrame Vehicle::getMostRecentFrame() const
{
  return buffer.back();
}

void Vehicle::resetVehicle()
{
  // We will make some lazy assumptions with regard to velocity magnitude in this function as this data
  // is not stored and is used to check lane availability and adjacency.
  buffer.clear();
  updated = false; 
  d_dot = 0.0;
  s_dot = 0.0;
  s_dot_dot = 0.0;
  d_dot_dot = 0.0;
  s_rel = 9999;
}

ExternalVehicle ExternalVehicle::makeFutureExtVehicle(double deltaT)
{
  VehicleFrame futureFrame = predictForward(deltaT);
  ExternalVehicle out(futureFrame);
  out.s_dot = s_dot_dot * deltaT;
  out.d_dot = d_dot_dot * deltaT;
  return out; 
}

void ExternalVehicle::setFutureSrel(VehicleFrame egoFuture)
{ 
  s_rel = s_relative(egoFuture.s, getMostRecentFrame().s);
}

VehicleFrame EgoVehicle::egoPredictForward(double deltaT, int tgtLane)
{
  // We will make some lazy assumptions with regard to velocity magnitude in this function as this data
  // is not stored and is used to check lane availability and adjacency.
  VehicleFrame egoFutureOut = predictForward(deltaT);
  egoFutureOut.d = double(tgtLane)*4.0 - 2.0;
  egoFutureOut.lane = getLane(egoFutureOut.d);
}
 
Vehicle EgoVehicle::makeFutureEgoVehicle(double deltaT,int tgtLane) 
{
  VehicleFrame futureFrame = egoPredictForward(deltaT,tgtLane);
  Vehicle out;
  out.resetVehicle(); 
  out.addFrame(futureFrame); 
  out.s_dot = s_dot_dot * deltaT;
  out.d_dot = d_dot_dot * deltaT;
  return out; 
}

// provide an estimated state for the vehicle at time delta T from most recent frame.
VehicleFrame Vehicle::predictForward(double deltaT)
{
  // We will make some lazy assumptions with regard to velocity magnitude in this function as this data
  // is not stored and is used to check lane availability and adjacency.
  VehicleFrame frameOut = getMostRecentFrame();

  frameOut.s += s_dot * deltaT + s_dot_dot * deltaT * deltaT;
  frameOut.d += d_dot * deltaT;
  frameOut.lane = getLane(frameOut.d);
  std::vector<double> xyvxvy = track->sd_to_xyv(frameOut.s, frameOut.d, s_dot, d_dot);
  frameOut.x = xyvxvy[0];
  frameOut.y = xyvxvy[1];
  frameOut.v_mag = sqrt(xyvxvy[2] * xyvxvy[2] + xyvxvy[3] * xyvxvy[3]);
  frameOut.yaw = atan2(xyvxvy[3], xyvxvy[2]);
  frameOut.dt = deltaT;
  return frameOut;
}

void VehicleField::updateVehicle(double s_rel, VehicleFrame tmpVehFrm)
{
  bool neighbor = (s_rel < searchAhead) && (s_rel > searchBehind);
  int id = tmpVehFrm.id;
  if (neighbor)
  {
    if (localCars.count(id) == 0)
    {
      localCars.emplace(std::make_pair(id, ExternalVehicle(tmpVehFrm)));
    }
    else
    {
      localCars.at(id).addFrame(tmpVehFrm);
    }
    localCars.at(id).updated = true;
    localCars.at(id).s_rel = s_rel;
  }
  else
  {
    if (localCars.count(id) != 0)
    {
      if (localCars.at(id).updated == true)
      {
        localCars.at(id).resetVehicle();
      }
    }
  }
};

void VehicleField::checkLaneRightCurrent(const VehicleFrame &egoNow)
{
  double my_s = egoNow.s;
  double my_lane = egoNow.lane;
  if (my_lane == 2)
  {
    std::cout << "Lane Right Unavailable" << std::endl;
  }
  for (auto car_iter = localCars.begin(); car_iter != localCars.end(); car_iter++)
  {
    if (car_iter->second.updated)
    {
      VehicleFrame tgtFrm = car_iter->second.getMostRecentFrame();
      if (tgtFrm.lane == my_lane + 1)
      {
        if (tgtFrm.s < (my_s + 4.0) && tgtFrm.s > (my_s - 4.0))
        {
          std::cout << "RIGHT LANE BLOCKED" << std::endl;
        }
      }
    }
  }
}

void VehicleField::checkLaneLeftCurrent(const VehicleFrame &egoNow)
{
  double my_s = egoNow.s;
  double my_lane = egoNow.lane;
  if (my_lane == 0)
  {
    std::cout << "Lane Left Unavailable" << std::endl;
  }
  for (auto car_iter = localCars.begin(); car_iter != localCars.end(); car_iter++)
  {
    if (car_iter->second.updated)
    {
      VehicleFrame tgtFrm = car_iter->second.getMostRecentFrame();
      if (tgtFrm.lane == my_lane - 1)
      {
        double s_rel = s_relative(my_s, tgtFrm.s);
        if (s_rel < 4.0 && s_rel > -4.0)
        {
          std::cout << "LEFT LANE BLOCKED" << std::endl;
        }
      }
    }
  }
}

bool VehicleField::checkAdjacentLaneOccupancy(const VehicleFrame &egoNow, const int lane)
{
  bool blocked = false; 
  double my_s = egoNow.s;
  for (auto car_iter = localCars.begin(); car_iter != localCars.end(); car_iter++)
  {
    if (car_iter->second.updated)
    {
      VehicleFrame tgtFrm = car_iter->second.getMostRecentFrame();
      if (tgtFrm.lane == lane)
      {
        double s_rel = s_relative(my_s, tgtFrm.s);
        if (s_rel < 4.0 && s_rel > -4.0)
        {
          blocked = true;
        }
      }
    }
  }
  return blocked;
}


double VehicleField::getVehicleSDist(int id)
{
  return localCars.at(id).s_rel;
}


int VehicleField::getFowardCar(int lane)
{
  double fwdDist = 9999.0;
  int fwdId = -1;
  for (auto car_iter = localCars.begin(); car_iter != localCars.end(); car_iter++)
  {
    if (car_iter->second.updated)
    {
      if (lane == car_iter->second.getMostRecentFrame().lane)
      {
        double s_rel = car_iter->second.s_rel;
        if (s_rel > 0) //if the car is ahead of mine
        {
          if (s_rel < fwdDist) // but closer than the fwdDist;
          {
            fwdDist = s_rel;
            fwdId = car_iter->first;
          }
        }
      }
    }
  }
  return fwdId;
}
double VehicleField::getVehicleSpeed(int id)
{
  return localCars.at(id).getMostRecentFrame().v_mag;
}

double VehicleField::getFrenetTimeToCollisionQuad(int id)
{
  if (ego_ptr->estimating == false)
  {
    return 999;
  }
  else if (localCars.find(id)->second.estimating == false)
  {
    return 999;
  }
  else
  {
    // create a polynomial comprised of the difference and solve for roots
    auto tgtPtr = localCars.find(id);
    double a = tgtPtr->second.s_dot_dot - ego_ptr->s_dot_dot;
    double b = tgtPtr->second.s_dot - ego_ptr->s_dot;
    double c = -1 * tgtPtr->second.s_rel;

    double discriminant = b * b - 4 * a * c; 

    if (discriminant > 0)
    {
      auto rts = getRoots(a, b, c, discriminant);
      if(rts[0] > 0 && rts[1] < 0)
      {
        return rts[0];
      }
      else if (rts[0] < 0 && rts[1] > 0)
      {
        return rts[1];
      }
      else if(rts[0] > 0 && rts[1]>0)
      { 
        if(rts[0]>rts[1])
        {
          return rts[0];
        }
        else
        {
          return rts[1];
        }
      }
    }
    else if (discriminant == 0)
    {
      auto rts = getRoots(a, b, c, discriminant);
      return rts[0];
    }
    else
    {
      return -1;
    }  
  }
}

  double VehicleField::getFrenetTimeToCollision(int id)
{
  if (ego_ptr->estimating == false)
  {
    return 999;
  }
  else if (localCars.find(id)->second.estimating == false)
  {
    return 999;
  }
  else
  {
    auto tgtPtr = localCars.find(id);
    double relSpd = ego_ptr->getMostRecentFrame().v_mag - tgtPtr->second.getMostRecentFrame().v_mag;
    double closingTime = tgtPtr->second.s_rel/relSpd;
    //std::cout<<"ct "<<closingTime;
    return closingTime;
  }
}

VehicleField VehicleField::makeFutureVehicleField(Vehicle egoFuture, double deltaT)
{
  VehicleField VFout(std::make_shared<Vehicle> (egoFuture)); 
  for (auto car_iter = localCars.begin(); car_iter != localCars.end(); car_iter++)
  { 
    if (car_iter->second.updated)
    {
      ExternalVehicle newVeh = car_iter->second.makeFutureExtVehicle(deltaT);
      newVeh.setFutureSrel(egoFuture.getMostRecentFrame());
      VFout.localCars.emplace(std::make_pair(car_iter->first, newVeh));
        
    }
  }
  return VFout; 
}