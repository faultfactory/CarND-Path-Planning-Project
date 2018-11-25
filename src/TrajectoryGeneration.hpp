#ifndef TRAJECTORY_GENERATION
#define TRAJECTORY_GENERATION

#include <memory>
#include "track.h"
#include "vehicles.hpp"

struct TrajectorySet
{
    std::vector<double> xPts;
    std::vector<double> yPts;
};

struct PositionHeading
{
    double x;
    double y;
    double yaw;
};

class TrajectoryGeneration
{
    static std::shared_ptr<Track> track;
    std::shared_ptr<EgoVehicle> egoPtr;
    VehicleFrame egoNow;
    PositionHeading ref_position;
    PathStatus prior;

    // Points for output
    TrajectorySet output; 

    // 
    public: 
    TrajectoryGeneration(std::shared_ptr<EgoVehicle> ep)
    {
        egoPtr=ep;
    }

    protected: 
    void clearPathVectors()
    {
        output.xPts.clear();
        output.yPts.clear();
    }
    void initializeStubTrajectory();
    void setStubTrajectory(PathStatus*);
    void resetTrajectoryData();
    TrajectorySet transformToVehicle(TrajectorySet);
    TrajectorySet transformToWorld(TrajectorySet);
};




#endif