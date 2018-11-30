#ifndef TRAJECTORY_GENERATION
#define TRAJECTORY_GENERATION

#include <memory>
#include "track.h"
#include "vehicles.hpp"

struct TrajectorySet
{
    std::vector<double> xPts;
    std::vector<double> yPts;
    
    void clear();
};

struct ReferenceState
{
    double x;
    double y;
    double yaw;
    double velocity;
};

class TrajectoryGeneration
{
    

    // 
    public: 
    TrajectoryGeneration(std::shared_ptr<EgoVehicle> ep) : egoPtr(ep),
                                                           pathCount(50),
                                                           targetLane(1){};


    protected: 
    bool priorPathValid; 
    const double systemCycleTime = 0.02; 
    int targetLane;
    double targetVelocity;
    static std::shared_ptr<Track> track;
    VehicleFrame egoNow;
    std::shared_ptr<EgoVehicle> egoPtr;
    ReferenceState refState;
    PathStatus prior;    
    // Points for path calculation
    TrajectorySet pathSeed; 

    //Path to output
    TrajectorySet outputPath;
    
    void clearPathVectors()
    {
        pathSeed.clear();
        outputPath.clear();
    }
    void initializeStubTrajectory();
    void setStubTrajectory(PathStatus*);
    void resetTrajectoryData();
    TrajectorySet transformToVehicle(TrajectorySet);
    TrajectorySet transformToWorld(TrajectorySet);
    void includePriorPathData();

    int pathCount; 
};

class TrajectorySplineBased : public TrajectoryGeneration
{
    TrajectorySplineBased(std::shared_ptr<EgoVehicle> ep) : TrajectoryGeneration (ep),
                                                            waypoints(3),
                                                            waypoint_s_increment(30.0){};
    tk::spline spline; 

    void setSpline();
    void setTargetLane(int);
    void setTargetVelocity(int);

    private:    
    int waypoints;
    double waypoint_s_increment;

    
    void setWaypointParameters(int,double);
    void generatePath();
    
};

#endif