#include "behavior.hpp"


Behavior::Behavior(shared_ptr<Vehicle> ep, shared_ptr<VehicleField> extp)
{
    // pointer to ego vehicle
    ego_ptr = ep;
    // pointer to external cars
    ext_ptr = extp;
}

void Behavior::printFwdDdot(int lane)
{
    int id = ext_ptr->getFowardCar(lane);
    if(id != -1 && ext_ptr->localCars.at(id).estimating )
    {
        std::cout<<ext_ptr->localCars.at(id).d_dot;
    }
    else
    {
        std::cout<<"NA";
    }
    std::cout<<" ";
}

void Behavior::printFwdDdotdot(int lane)
{
    int id = ext_ptr->getFowardCar(lane);
    if(id != -1 && ext_ptr->localCars.at(id).estimating )
    {
        std::cout<<ext_ptr->localCars.at(id).d_dot_dot;
    }
    else
    {
        std::cout<<"NA";
    }
    std::cout<<" ";
}


double Behavior::getLanePresentCost(int lane,int currentLane,VehicleFrame egoNow)
{
    double cost = 0.0; 
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
    if(lane == 0)
    {
        cost +=201;
    }
    if(lane == 2)
    {
        cost +=250;
    }
    // I want to add a small cost to doing a lane change at all
    // to prevent oscillations and strange behavior
    if( lane != currentLane)
    {
        cost += 150; 
    }
    // If a car is in the adjacent lane, the cost to go to that lane
    // should be orders of magnitude higher
    if (ext_ptr->checkAdjacentLaneOccupancy(egoNow,lane))
    {
        cost += 100000;
    }
    // Lanes with cars ahead of me will be costed accordingly. 
    int ah_id = ext_ptr->getFowardCar(lane);
    if(ah_id != -1)
    {
        cost += 11 * (ext_ptr->searchAhead - ext_ptr->getVehicleSDist(ah_id));
        cost += 11 * (spd_lim - ext_ptr->getVehicleSpeed(ah_id));
    }
    return cost;
}

int Behavior::getLowestCostLane()
{   
    std::vector<double> costs;
    double cost;
    VehicleFrame egoNow = ego_ptr->getMostRecentFrame();
    int currentLane = egoNow.lane; 
    for(auto ln = lanes.begin(); ln!=lanes.end(); ln++)
    {          
        costs.push_back(getLanePresentCost(*ln,currentLane,egoNow));

    }

    int min_pos = distance(costs.begin(),min_element(costs.begin(),costs.end()));

    // for(auto i: costs)
    // {
    //     std::cout<<i<<" ";
    // }
    // std::cout<<std::endl;
    // This blocks double lane changes.
    // it also helps in situations where the center lane is not 
    // the best lane but being there allows for a transition to 
    // a clear edge lane;
    double bridgeLaneCost = 300;
    if(abs(min_pos-egoNow.lane)>1)
    {
        if((bridgeLaneCost+costs[egoNow.lane])>costs[1])
        {
            min_pos = 1; 
        }
        else
        {
            min_pos = egoNow.lane;
        }
        
        }
        return min_pos;
}




void Behavior::keepLane(double *tgt_vel)
{   
    int ahead_id = ext_ptr->getFowardCar(ego_ptr->getMostRecentFrame().lane);
    setFollowingSpeed(tgt_vel,ahead_id);
};


void Behavior::setFollowingSpeed(double *tgt_vel, int ahead_id)
{   
    if(ahead_id != -1)
    {	
        
        double ahead_s = ext_ptr->getVehicleSDist(ahead_id);
        double ttc = ext_ptr->getFrenetTimeToCollision(ahead_id);
        double forwardCarSpeed = ext_ptr->getVehicleSpeed(ahead_id);

        if(ttc>0)
        {
            //
        }
        if(ttc>0 && ttc<2.0 || ahead_s<4)
        {
            *tgt_vel = max(0.0,forwardCarSpeed-5.0);
        }
        if(ahead_s>10 && ahead_s<15)
        {
            *tgt_vel = min(0.8*forwardCarSpeed,spd_lim);
        }
        if(ahead_s>15 && ahead_s<20)
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

void Behavior::setLaneChangeSpeed(double *tgt_vel, int tgtLane)
{
    int ahead_id_old = ext_ptr->getFowardCar(ego_ptr->getMostRecentFrame().lane);
    int ahead_id_new = ext_ptr->getFowardCar(tgtLane);
    double currentSpeed = *tgt_vel; 


    if (ahead_id_old == -1 && ahead_id_new == -1)
    {
        *tgt_vel = spd_lim;
    }
    else if(ahead_id_new != -1 && ahead_id_old !=-1)
    {

        double old_lane_speed = ext_ptr->getVehicleSpeed(ahead_id_old);
        double new_lane_speed = ext_ptr->getVehicleSpeed(ahead_id_new);
        double ahead_s_new =ext_ptr->getVehicleSDist(ahead_id_new);
        double ahead_s_old =ext_ptr->getVehicleSDist(ahead_id_old);
        double ahead_s = min(ahead_s_new,ahead_s_old);
        double ttc = ext_ptr->getFrenetTimeToCollision(ahead_id_old);
        double forwardCarSpeed = max(old_lane_speed,new_lane_speed);

        if (ttc > 0)
        {
            //
        }
        if (ttc > 0 && ttc < 2.0 || ahead_s < 2)
        {
            *tgt_vel = max(0.0, forwardCarSpeed - 5.0);
        }
        if (ahead_s > 10 && ahead_s < 15)
        {
            *tgt_vel = min(0.8 * forwardCarSpeed, spd_lim);
        }
        if (ahead_s > 15 && ahead_s < 20)
        {
            *tgt_vel = min(forwardCarSpeed, spd_lim);
        }
        if (ahead_s > 50 && ahead_s < 60)
        {
            *tgt_vel = min(1.2 * forwardCarSpeed, spd_lim);
        }
        if (ahead_s > 60.0)
        {
            *tgt_vel = spd_lim;
        }
    }
    else
    {
        *tgt_vel = spd_lim;
    }
};
