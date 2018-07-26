#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
//#include "Eigen-3.3/Eigen/Core"
//#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "helpers.hpp"
#include "tic_toc.h"
#include "track.h"
#include "vehicles.hpp"
#include "behavior.hpp"

using namespace std;

extern Track track;

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.


// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
  
}
double loop_time_ms = 1000.0; 
Track track = Track("../data/highway_map.csv");

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors

  // Waypoint map to read from
  ///std::string map_file_ = ;
  
  tic();
    
  double max_s = 6945.554;

	int lane = 1; 
	Vehicle egoVeh;
	double ref_vel = 0.0;
	double tgt_vel = spd_lim;
	VehicleField extVehs(&egoVeh);
	Behavior plan(&egoVeh,&extVehs);
	bool lane_change = false;
	
	h.onMessage([&lane_change,&lane, &ref_vel, &tgt_vel,&egoVeh, &extVehs, &plan](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,uWS::OpCode opCode) {
		// "42" at the start of the message means there's a websocket message event.
		// The 4 signifies a websocket message
		// The 2 signifies a websocket event
		//auto sdata = string(data).substr(0, length);
		//cout << sdata << endl;

		//std::cout<<std::endl;
		if (length && length > 2 && data[0] == '4' && data[1] == '2')
		{
			auto s = hasData(data);

			if (s != "")
			{
				auto j = json::parse(s);
				
				
				string event = j[0].get<string>();

				if (event == "telemetry")
				{
					// Capture loop time for state calcs
					loop_time_ms = toc();
					// reset clock
					tic();
				    egoVeh.addEgoFrame(j);
					//std::cout<<"sdot "<<egoVeh.s_dot<<" ";
					VehicleFrame egoNow = egoVeh.getMostRecentFrame();
					//std::cout<<egoNow.v_mag;

					// Previous path data given to the Planner
					auto previous_path_x = j[1]["previous_path_x"];
					auto previous_path_y = j[1]["previous_path_y"];
					// Previous path's end s and d values
					double end_path_s = j[1]["end_path_s"];
					double end_path_d = j[1]["end_path_d"];

					// Sensor Fusion Data, a list of all other cars on the same side of the road.
					vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
					
					extVehs.updateLocalCars(egoNow,sensor_fusion);
					

					int prev_size = previous_path_x.size();

					for(auto ln = plan.lanes.begin(); ln!=plan.lanes.end(); ln++)
					{
						plan.printFwdDdotdot(*ln);
					}
					


					std::cout<<std::endl;
					// collision avoidance code: 
					if(!lane_change)
					{
						lane = plan.getLowestCostLane();
					}
					else
					{
						plan.setLaneSpeed(&tgt_vel,lane);
					}
					lane_change = (lane!=egoNow.lane);
					if(!lane_change)
					{
						plan.keepLane(&tgt_vel); 
					}
					
					// Create vector of new points to fil
					vector<double> ptsx;
					vector<double> ptsy;

					// Find current car state; 
					double ref_x = egoNow.x;
					double ref_y = egoNow.y;
					double ref_yaw = egoNow.yaw;

					// Assure tangency for the current sate.
					// if the previous was almost empty reset
					if (prev_size < 2)
					{
						// Generate two points that align with c urrent state;
						double prev_car_x = egoNow.x - cos(egoNow.yaw);
						double prev_car_y = egoNow.y - sin(egoNow.yaw);

						ptsx.push_back(prev_car_x);
						ptsx.push_back(egoNow.x);

						ptsy.push_back(prev_car_y);
						ptsy.push_back(egoNow.y);
					}
					else
					{ 
						ref_x = previous_path_x[prev_size - 1];
						ref_y = previous_path_y[prev_size - 1];

						double ref_x_prev = previous_path_x[prev_size - 2];
						double ref_y_prev = previous_path_y[prev_size - 2];
						ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

						ptsx.push_back(ref_x_prev);
						ptsx.push_back(ref_x);

						ptsy.push_back(ref_y_prev);
						ptsy.push_back(ref_y);
					}

					int wPoints = 3;
					int sIncrement = 30;
					if(lane_change)
					{
						sIncrement = 50;
					}
					

					
					for (int i = 1; i < (wPoints + 1); i++)
					{
						// This does not work unless you add lane to the function line
						vector<double> next_wp = track.sd_to_xy(egoNow.s + (sIncrement * i), (2 + 4 * lane));
						ptsx.push_back(next_wp[0]);
						ptsy.push_back(next_wp[1]);
						
					}

					// Coordinate rotation to vehicle frame
					std::vector<double> safex;
					std::vector<double> safey;

					double shift_x = ptsx[0] - ref_x;
					double shift_y = ptsy[0] - ref_y;
					
					safex.push_back(shift_x * cos(0 - ref_yaw) - (shift_y)*sin(0 - ref_yaw));
					safey.push_back(shift_x * sin(0 - ref_yaw) + (shift_y)*cos(0 - ref_yaw));

					for (int i = 1; i < ptsx.size(); i++)
					{
						
						double shift_x = ptsx[i] - ref_x;
						double shift_y = ptsy[i] - ref_y;
						double testx=shift_x * cos(0 - ref_yaw) - (shift_y)*sin(0 - ref_yaw);
						if(safex[i-1]!=testx)
						{
							safex.push_back(shift_x * cos(0 - ref_yaw) - (shift_y)*sin(0 - ref_yaw));
							safey.push_back(shift_x * sin(0 - ref_yaw) + (shift_y)*cos(0 - ref_yaw));
						}
					}
					tk::spline s;
					s.set_points(safex, safey);

					// Create display spline.
					vector<double> next_x_vals;
					vector<double> next_y_vals;

					// Re-use prior path points that were not consumed by the car.

					for (int i = 0; i < previous_path_x.size(); i++)
					{
						next_x_vals.push_back(previous_path_x[i]);
						next_y_vals.push_back(previous_path_y[i]);
					}

					// how to break the spline points;
					double target_x = 30.0;
					double target_y = s(target_x);
					double target_dist = sqrt((target_x * target_x) + (target_y * target_y));

					double x_add_on = 0;
					// Create configurable number for path points
					int pathPtCount = 50;
					// Fill the rest of the points.
					for (int i = 1; i <= pathPtCount - previous_path_x.size(); i++)
					{
						double veldiff = ref_vel-tgt_vel;
						bool change = fabs(veldiff)>vel_inc;
	
						if(change)
						{
							if(veldiff<vel_inc)
							{
								ref_vel+=vel_inc;
							}
							else if(veldiff>vel_inc)
							{
								ref_vel-=vel_inc;
							}
						} 
						
						double N = (target_dist / (0.02 * ref_vel));
						double x_point = x_add_on + (target_x) / N;
						double y_point = s(x_point);

						x_add_on = x_point;

						double x_ref = x_point;
						double y_ref = y_point;

						//transform system back to world coordinates after defining path spline

						x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
						y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

						x_point += ref_x;
						y_point += ref_y;

						next_x_vals.push_back(x_point);
						next_y_vals.push_back(y_point);

					}
					json msgJson;
					// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
					msgJson["next_x"] = next_x_vals;
					msgJson["next_y"] = next_y_vals;

					auto msg = "42[\"control\"," + msgJson.dump() + "]";
					
					//this_thread::sleep_for(chrono::milliseconds(1000));
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
				
			}
			else
			{
				// Manual driving
				std::string msg = "42[\"manual\",{}]";
				ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
			
		}
		
	});

	// We don't need this since we're not using HTTP but if it's removed the
	// program
	// doesn't compile :-(
	h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
					   size_t, size_t) {
		const std::string s = "<h1>Hello world!</h1>";
		if (req.getUrl().valueLength == 1)
		{
			res->end(s.data(), s.length());
		}
		else
		{
			// i guess this should be done more gracefully?
			res->end(nullptr, 0);
		}
	});

	h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
		std::cout << "Connected!!!" << std::endl;
	});

	h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
						   char *message, size_t length) {
 		ws.close();
		std::cout << "Disconnected" << std::endl;
	});

	int port = 4567;
	if (h.listen(port))
	{
		std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
