#ifndef INPUTHANDLER
#define INPUTHANDLER
#include "json.hpp"
#include "vehicles.hpp"
#include "tic_toc.h"

class InputHandler
{
    double loop_time_ms = 1000.0;
    static std::shared_ptr<Track> track;
    std::shared_ptr<EgoVehicle> egoVeh;
    std::shared_ptr<VehicleField> extVehs;
    
    void updateExternalVehicles(std::vector<std::vector<double>> *incomingData);

    public:
    InputHandler(std::shared_ptr<EgoVehicle> egoV, std::shared_ptr<VehicleField> extVs)
    {
        egoVeh=egoV;
        extVehs=extVs;
    }
    
    void processInputMessage(nlohmann::json j);

    private:
    EgoFrame makeEgoFrame(nlohmann::json*);
    VehicleFrame makeExternalFrame(std::vector<double> v);    
};

#endif