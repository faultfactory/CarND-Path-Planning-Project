#include "InputHandler.hpp"

void InputHandler::processInputMessage(nlohmann::json j)
{
  loop_time_ms = toc();
  tic();

  EgoFrame egoFrm;
  // Creates vehicle frame for Ego Vehicle
  // need to verify the output of json::parse() and adjust appropriately
  egoFrm.id = -1;
  egoFrm.x = j[1]["x"];
  egoFrm.y = j[1]["y"];
  egoFrm.s = j[1]["s"];
  egoFrm.d = j[1]["d"];
  egoFrm.yaw = deg2rad(j[1]["yaw"]);
  egoFrm.v_mag = (j[1]["speed"]);
  egoFrm.v_mag *= 0.44704; // ego speed is delivered in mph but everything else is in meters, we will convert here and throw away mph
  egoFrm.lane = getLane(egoFrm.d);
  egoFrm.dt = loop_time_ms / 1000.0;

  // Previous path data given to the Planner
  egoFrm.pathStatus.previous_path_x = j[1]["previous_path_x"];
  egoFrm.pathStatus.previous_path_y = j[1]["previous_path_y"];
  // Previous path's end s and d values
  egoFrm.pathStatus.end_path_s = j[1]["end_path_s"];
  egoFrm.pathStatus.end_path_d = j[1]["end_path_d"];
  
  egoVeh->addEgoFrame(egoFrm);

  vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

  updateExternalVehicles(&sensor_fusion);
};



VehicleFrame InputHandler::makeExternalFrame(std::vector<double> v)
{
  VehicleFrame out;
  // Creates vehicle frame for other vehicle
  out.id = int(v[0]);

  out.x = v[1];
  out.y = v[2];

  double vx = v[3];
  double vy = v[4];

  out.v_mag = sqrt(vx * vx + vy * vy);
  out.yaw = atan2(vy, vx);

  out.s = v[5];
  out.d = v[6];
  out.lane = getLane(out.d);
  out.dt = loop_time_ms / 1000.0;
  return out;
}; //sensor fusion construct


void InputHandler::updateExternalVehicles(std::vector<std::vector<double>> *incomingData)
{
  VehicleFrame egoNow =  egoVeh->getMostRecentFrame();
  double myS = egoNow.s;
  //double myS = egoNow.s;
  for (auto i = incomingData->begin(); i != incomingData->end(); i++)
  {
    VehicleFrame tmpVehFrm = makeExternalFrame(*i);

    double s_rel = s_relative(egoNow.s, tmpVehFrm.s);
    extVehs->updateVehicle(s_rel,tmpVehFrm);

  }
}