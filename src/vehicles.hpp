#ifndef VEHICLES
#define VEHICLES

#include "math.h"
#include "helpers.hpp"
#include "json.hpp"
#include <deque>
#include <map>
#include "track.h"
#include <memory>



struct PathStatus
{
  nlohmann::basic_json<> previous_path_x;
  nlohmann::basic_json<> previous_path_y;

  double end_path_s;
  double end_path_d;
};

struct VehicleFrame
{
  int id;       // sensor fusion has an ID. I'm just going to set it to -1 for the ego car.
  double x;     // X position in m
  double y;     // Y position in m
  double s;     // Frenet Coordinate
  double d;     // Frenet Coorindate d
  double v_mag; // magnitude of velocity in m/s
  double yaw;   // yaw direction
  int lane;     //
  double dt;    // seconds since last frame;
};

struct EgoFrame : public VehicleFrame
{
  PathStatus pathStatus;
};




//////////////////





class Vehicle
{
  // I am aware that this should be abstracted into a parent and two subclasses.
  // "God punishes those who optimze early"
  static std::shared_ptr<Track> track;
  const int bufferMax = 50;
  const int estimationMin = 20;
  std::deque<VehicleFrame> buffer;

public:
  Vehicle(VehicleFrame);
  Vehicle();

  void resetVehicle();
  // These parameters might be most useful for determining collisions;
  double s_rel; // Relative s coordinate;
  double d_dot;
  double d_dot_dot;
  double s_dot;
  double s_dot_dot;

  bool estimating; // value to tell external members to use or to ignore these state values
  bool updated;    // tells Vehicle field whether this has been updated in this most recent frame

  VehicleFrame predictForward(double deltaT);
  VehicleFrame getMostRecentFrame() const;

  void addFrame(VehicleFrame);
  void estimateValues();
};

class ExternalVehicle : public Vehicle
{
  public:
  ExternalVehicle(VehicleFrame vf) : Vehicle(vf){};
  ExternalVehicle makeFutureExtVehicle(double deltaT);
  void addExternalFrame(VehicleFrame);
  void setFutureSrel(VehicleFrame egoFuture);
};

class EgoVehicle : public Vehicle
{
  PathStatus previousPath;
  public: 
  PathStatus getPreviousPath(){return previousPath;};
  VehicleFrame egoPredictForward(double deltaT, int tgtLane);
  Vehicle makeFutureEgoVehicle(double deltaT, int tgtLane);
  void addEgoFrame(EgoFrame);
};





/////////////////// 





//TODO: Consider breaking out the forward prediction components into a prediction class
//      This would create better separation of concerns.
class VehicleField
{
  std::shared_ptr<Vehicle> ego_ptr;

public:
  const double searchAhead = 70.0;
  const double searchBehind = -70.0;
  const double max_s = 6945.554;

  std::map<int, ExternalVehicle> localCars;

public:
  VehicleField(std::shared_ptr<Vehicle> ep)
  {
    ego_ptr = ep;
  }

  bool checkAdjacentLaneOccupancy(const VehicleFrame &egoNow, const int lane);
  int getFowardCar(int lane);
  double getFrenetTimeToCollisionQuad(int id);
  double getFrenetTimeToCollision(int id);
  double getVehicleSpeed(int id);
  double getVehicleSDist(int id);

  VehicleField makeFutureVehicleField(Vehicle egoFuture, double deltaT);
  void updateVehicle(double, VehicleFrame);

  // Printer functions to prototype algorithms and output information to terminal
  void checkLaneRightCurrent(const VehicleFrame &egoNow);
  void checkLaneLeftCurrent(const VehicleFrame &egoNow);
};

#endif