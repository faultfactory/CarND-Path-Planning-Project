#ifndef BEHAVIOR
#define BEHAVIOR

#include "vehicles.hpp"

struct Plan
{
    int targetLane;
    double targetVelocity;
};


class Behavior
{
    // pointer to ego vehicle
    Vehicle* ego_ptr;
    // pointer to external cars
    VehicleField* ext_ptr;

public:

    Behavior(Vehicle* ep, VehicleField* extp);
    struct Plan createBehaviorPlan();
    std::vector<int> lanes{0,1,2};

    void keepLane(double*);
    int getLowestCostLane();
    void setLaneSpeed(double *tgt_vel,int lane);
private:
    
    void setFollowingSpeed(double *tgt_vel, int ahead_id);
    double getLaneCost();

public:



};


#endif  