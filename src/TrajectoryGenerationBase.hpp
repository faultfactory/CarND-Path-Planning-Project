#ifndef TRAJECTORY_GENERATION_BASE
#define TRAJECTORY_GENERATION_BASE

#include <memory>
#include "track.h"
#include "vehicles.hpp"
#include "Eigen-3.3/Eigen/Dense"

struct TrajectorySet
{
    std::vector<double> xPts;
    std::vector<double> yPts;

    void concatenate(TrajectorySet);
    void clear();
};

struct TrajectoryState
{
    double x;
    double y;
    double yaw;
    double velocity;
};

class TrajectoryGeneration
{
  public:
    TrajectoryGeneration(std::shared_ptr<EgoVehicle> ep) : egoPtr(ep),
                                                           pathCount(50),
                                                           targetLane(1){};

    ~TrajectoryGeneration(){};

    void resetTrajectoryData();
    TrajectorySet generateNextPath();
    void setTargetLane(int);
    void setTargetVelocity(double);
  protected:
    bool priorPathValid = true; // added for including eventual option to abandon path

    const double systemCycleTime = 0.02;
    int targetLane;
    double targetVelocity;

    static std::shared_ptr<Track> track;
    VehicleFrame egoNow;
    std::shared_ptr<EgoVehicle> egoPtr;
    double pathReferenceVelocity;

    TrajectoryState refState;
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
 
    virtual void generatePath(){};
    int pathCount;

    double endSVelocity;
    void calculateEndSVelocity();
};

#endif