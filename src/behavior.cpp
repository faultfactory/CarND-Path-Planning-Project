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
        if( *ln != currentLane)
        {
            cost += 10; 
        }
        if (ext_ptr->checkAdjacentLaneOccupancy(egoNow,*ln))
        {
            cost += 1000;
        }
        int ah_id = ext_ptr->getFowardCar(*ln);
        if(ah_id != -1)
        {
            cost += 100 * ext_ptr->getVehicleSDist(ah_id);
            cost += 10 * (spd_lim - ext_ptr->getVehicleSpeed(ah_id));
        }
        
        costs.push_back(cost);

    }
    int min_pos = distance(costs.begin(),min_element(costs.begin(),costs.end()));
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
        std::cout<<" AID "<<ahead_id<<std::endl;
        if(ttc>0)
        {
            //
        }
        if(ttc>0 && ttc<2.0 || ahead_s<10)
        {
            *tgt_vel = max(0.0,forwardCarSpeed-5.0);
        }
        if(ahead_s>10 && ahead_s<20)
        {
            *tgt_vel = 0.8*forwardCarSpeed;
        }
        if(ahead_s>20 && ahead_s<40)
        {
            *tgt_vel = forwardCarSpeed;
        }
        if(ahead_s>50 && ahead_s<60)					
        {
            *tgt_vel = 1.2*forwardCarSpeed;
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
