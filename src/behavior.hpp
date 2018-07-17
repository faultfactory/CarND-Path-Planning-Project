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
    int lanes[3]= {0,1,2};

    void keepLane(double*);

private:
    int getLowestCostLane();
    double getLaneCost();
    

    












public:

};


#endif