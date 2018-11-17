#ifndef BEHAVIOR
#define BEHAVIOR

#include "vehicles.hpp"

//TODO: Consider making this class a singleton. You should only have a single instance of this 
//      Unless multiple types of Behavior plans are running to compare performance
class Behavior
{
    // pointer to ego vehicle
    std::shared_ptr<Vehicle> ego_ptr;
    // pointer to external cars
    std::shared_ptr<VehicleField> ext_ptr;

public:
    bool lane_change = false; // Indicate if plan has decided to change lanes;
    Behavior(std::shared_ptr<Vehicle> ep, std::shared_ptr<VehicleField> extp);
    const std::vector<int> lanes{0,1,2};

    void keepLane(double*);
    int getLowestCostLane();
    double checkLaneFuture(int lane,double forwardTime);
    double getLanePresentCost(int lane,int currentLane,VehicleFrame egoNow); 
    void setLaneChangeSpeed(double *tgt_vel, int tgtLane);
    void runBehaviorPlan();
    
private:    
    void setFollowingSpeed(double *tgt_vel, int ahead_id);

public:
// Print functions for developing algo
    void printFwdDdot(int);
    void printFwdDdotdot(int);

};


#endif  