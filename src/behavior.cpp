#include "behavior.hpp"


Behavior::Behavior(Vehicle* ep, VehicleField* extp)
{
    // pointer to ego vehicle
    ego_ptr = ep;
    // pointer to external cars
    ext_ptr = extp;
}


int Behavior::getLowestCostLane()
{   
    std::vector<double> costs;
    double cost;
    VehicleFrame egoNow = ego_ptr->getMostRecentFrame();
    int currentLane = egoNow.lane; 
    for(auto ln = lanes.begin(); ln!=lanes.end(); ln++)
    {
        cost = 0; 
        // I am adding a static cost function focusing on the center lane
        // for  3 reasons:
        // 1. There is a defect in the simulator at one point
        // telling me that i am outside the lane while i am clearly
        // inside it.This only happens in the outer most right lane. 
        // 2. From a legality and driver courtesty standpoint, it is 
        // frowned upon to pass on the right. Although many drivers
        // do not follow this rule if we are going to code behaviors,
        // we should code good ones. This means we should not linger in the 
        // left lane.
        // 3. Crusing in the center lane allows us more options for passing
        // without creating complex multi-step manuvers. 
        if(*ln == 0)
        {
            cost +=150;
        }
        if(*ln == 2)
        {
            cost +=300;
        }
        // I want to add a small cost to doing a lane change at all
        // to prevent oscillations and strange behavior
        if( *ln != currentLane)
        {
            cost += 200; 
        }
        // If a car is in the adjacent lane, the cost to go to that lane
        // should be orders of magnitude higher
        if (ext_ptr->checkAdjacentLaneOccupancy(egoNow,*ln))
        {
            cost += 100000;
        }
        // Lanes with cars ahead of me will be costed accordingly. 
        int ah_id = ext_ptr->getFowardCar(*ln);
        if(ah_id != -1)
        {
            cost += 10 * (150.0 - ext_ptr->getVehicleSDist(ah_id));
            cost += 10 * (spd_lim - ext_ptr->getVehicleSpeed(ah_id));
        }
        
        
        costs.push_back(cost);

    }

    int min_pos = distance(costs.begin(),min_element(costs.begin(),costs.end()));

    for(auto i: costs)
    {
        std::cout<<i<<" ";
    }
    std::cout<<std::endl;
    // This corrects for if the cente
    if(abs(min_pos-egoNow.lane)>1)
    {
        if(costs[egoNow.lane]>costs[1])
        {
            min_pos = 1; 
        }
        else
        {
            min_pos =egoNow.lane;
        }
        
        }
        return min_pos;
}




void Behavior::keepLane(double *tgt_vel)
{   
    int ahead_id = ext_ptr->getFowardCar(ego_ptr->getMostRecentFrame().lane);
    if(ahead_id != -1)
    {	
        
        double ahead_s = ext_ptr->getVehicleSDist(ahead_id);
        double ttc = ext_ptr->getFrenetTimeToCollision(ahead_id);
        double forwardCarSpeed = ext_ptr->getVehicleSpeed(ahead_id);

        if(ttc>0)
        {
            //
        }
        if(ttc>0 && ttc<2.0 || ahead_s<8)
        {
            *tgt_vel = max(0.0,forwardCarSpeed-5.0);
        }
        if(ahead_s>10 && ahead_s<15)
        {
            *tgt_vel = min(0.8*forwardCarSpeed,spd_lim);
        }
        if(ahead_s>20 && ahead_s<30)
        {
            *tgt_vel = min(forwardCarSpeed,spd_lim);
        }
        if(ahead_s>50 && ahead_s<60)					
        {
            *tgt_vel = min(1.2*forwardCarSpeed,spd_lim);
        }
        if(ahead_s>60.0)
        {
            *tgt_vel = spd_lim;
        }
    }
    else
    {
        *tgt_vel = spd_lim;
    }




};
