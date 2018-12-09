#ifndef TRAJECTORY_GENERATION
#define TRAJECTORY_GENERATION

#include <memory>
#include "track.h"
#include "vehicles.hpp"
#include "Eigen-3.3/Dense"

struct TrajectorySet
{
    std::vector<double> xPts;
    std::vector<double> yPts;
    
    void concatenate(TrajectorySet);
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


    void resetTrajectoryData();

    protected: 
    bool priorPathValid = true; // added for including eventual option to abandon path 
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
    void initializeStubTrajectoryFromCurrent();
    void setStubTrajectory();
    
    TrajectorySet transformToVehicle(TrajectorySet);
    TrajectorySet transformToWorld(TrajectorySet);
    void includePriorPathData();

    int pathCount; 
};

class TrajectorySplineBased : public TrajectoryGeneration
{
    public: 
    TrajectorySplineBased(std::shared_ptr<EgoVehicle> ep) : TrajectoryGeneration (ep),
                                                            waypoints(3),
                                                            waypoint_s_increment(30.0){};

    virtual ~TrajectorySplineBased(){};
    TrajectorySet generateNextPath();
    void setSpline();
    void setTargetLane(int);
    void setTargetVelocity(double);

    private:    
    int waypoints;
    double waypoint_s_increment;
    double pathReferenceVelocity;
    void updateReferenceVelocity();
    void generatePath();  
    tk::spline spline;     
};

class TrajectoryJMT : public TrajectoryGeneration
{
    double manueverDuration = systemCycleTime*pathCount;
    
    public: 
    TrajectoryJMT(std::shared_ptr<EgoVehicle> ep) : TrajectoryGeneration (ep),
                                                            waypoints(3),
                                                            waypoint_s_increment(30.0){};

    virtual ~TrajectoryJMT(){};
    std::vector<double> SingleAxisJMT(std::vector<double> start, std::vector<double> end, double T);
    TrajectorySet generateNextPath();
    void setTargetLane(int);
    void setTargetVelocity(double);

    private:    
    int waypoints;
    double waypoint_s_increment;
    double pathReferenceVelocity;
    void updateReferenceVelocity();
    void generatePath();  
};

#endif