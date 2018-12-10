#ifndef TRAJECTORY_GENERATION_CHILDREN
#define TRAJECTORY_GENERATION_CHILDREN

#include "TrajectoryGenerationBase.hpp"



class TrajectorySplineBased : public TrajectoryGeneration
{
    public: 
    TrajectorySplineBased(std::shared_ptr<EgoVehicle> ep) : TrajectoryGeneration (ep),
                                                            waypoints(3),
                                                            waypoint_s_increment(30.0){};

    virtual ~TrajectorySplineBased(){};
    
    void setSpline();
    void setTargetLane(int);
    void setTargetVelocity(double);

    private:    
    int waypoints;
    double waypoint_s_increment;
    double pathReferenceVelocity;
    void updateReferenceVelocity();

    tk::spline spline;

    protected:
    void generatePath() override;       
};

struct SingleAxisState
{
    double pos;
    double vel;
    double acc;
};


struct JMTEndConstraint
{
    struct SingleAxisState x;
    struct SingleAxisState y;
};

typedef std::vector<double> JMTCoeffs_t;

struct JMTPathDefinition
{
    JMTCoeffs_t x;
    JMTCoeffs_t y;
};

class TrajectoryJMT : public TrajectoryGeneration
{
    
    TrajectoryState targetEndState;

    JMTEndConstraint start;
    JMTEndConstraint end;
    JMTPathDefinition computedPath;
    int consumedIncrements;
    void getNewPathCount();
    double polyEval(JMTCoeffs_t,double);
    void createEndConstraint();
    void createStartConstraint();
    JMTCoeffs_t differentiate(JMTCoeffs_t);

    double endpointDistance;
   
    public: 
    TrajectoryJMT(std::shared_ptr<EgoVehicle> ep) : TrajectoryGeneration (ep),
                                                    endpointDistance(30.0){};

    virtual ~TrajectoryJMT(){};
    JMTCoeffs_t singleAxisJMT(SingleAxisState start, SingleAxisState end, double T);
    void calculateJMTPath();

    void setTargetLane(int);
    void setTargetVelocity(double);

    protected:
    void generatePath() override; 
         
};

#endif