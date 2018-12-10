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
#include "track.h"
#include "vehicles.hpp"
#include "InputHandler.hpp"
#include "behavior.hpp"
#include <memory>
#include "TrajectoryGenerationChildren.hpp"


using namespace std;

auto trackExternal = std::make_shared<Track>("../data/highway_map.csv");
std::shared_ptr<Track> Vehicle::track = trackExternal;         // Static member of Vehicle class
std::shared_ptr<Track> InputHandler::track = trackExternal;    // Static member of InputHandler class
std::shared_ptr<Track> TrajectoryGeneration::track = trackExternal;  // Static member of TrajectoryGeneration class

// for convenience
using json = nlohmann::json;


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


int main() {
  uWS::Hub h;
	

    
    double max_s = 6945.554;
	double tgt_vel = spd_lim;
	int lane = 1; 


	std::shared_ptr<EgoVehicle> egoVeh = std::make_shared<EgoVehicle>();
	std::shared_ptr<VehicleField> extVehs=std::make_shared<VehicleField>(egoVeh);
	InputHandler inputHandler(egoVeh,extVehs);
	TrajectorySplineBased trajectory(egoVeh);
	Behavior plan(egoVeh,extVehs);

	
	h.onMessage([&lane, &tgt_vel,&egoVeh, &extVehs, &plan, &inputHandler, &trajectory](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,uWS::OpCode opCode) {
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
					inputHandler.processInputMessage(j);
					VehicleFrame egoNow = egoVeh->getMostRecentFrame();

					if(!plan.lane_change)
					{
						lane = plan.getLowestCostLane();
					}
					else
					{
						plan.setLaneChangeSpeed(&tgt_vel,lane);
					}
					
					plan.lane_change = (lane!=egoNow.lane);
					if(!plan.lane_change)
					{
						plan.keepLane(&tgt_vel); 
					}

					trajectory.resetTrajectoryData();
					trajectory.setTargetVelocity(tgt_vel);
					trajectory.setTargetLane(lane);
					
					/// Not incorporated yet
					// int wPoints = 3;
					// int sIncrement = 30;
					// if(plan.lane_change)
					// {
					// 	sIncrement = 50;
					// }
					trajectory.setSpline();

					auto nextPath = trajectory.generateNextPath();

					json msgJson;
					// TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
					msgJson["next_x"] = nextPath.xPts;
					msgJson["next_y"] = nextPath.yPts;

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


	// Need to play with this further to allow for other IP's to be 
	// interrogated. Ideally the simlator would run externally so the 
	// internal visualizer can run on this system. 
	std::string hostIP("127.0.0.1");
	//std::string hostIP("192.168.1.62");
	const char * host = hostIP.c_str();
	int port = 4567;
	if (h.listen(host, port))
	{
		std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
