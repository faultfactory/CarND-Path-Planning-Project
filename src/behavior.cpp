#include "behavior.hpp"


Behavior::Behavior(Vehicle* ep, VehicleField* extp)
{
    // pointer to ego vehicle
    ego_ptr = ep;
    // pointer to external cars
    ext_ptr = extp;
}


void Behavior::keepLane(double *tgt_vel)
{   

    
    int ahead_id = ext_ptr->getFowardCar(1);
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
            *tgt_vel = 22.1;
        }
    }
    else
    {
        *tgt_vel = 22.1;
    }



};
