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

typedef std::array<double,6> JMTCoeffs_t;

struct JMTPathDefinition
{
    JMTCoeffs_t x;
    JMTCoeffs_t y;
};

class TrajectoryJMT : public TrajectoryGeneration
{

    JMTEndConstraint start;
    JMTEndConstraint end;
    JMTPathDefinition computedPath;
    int consumedIncrements;
    void getNewPathCount();
    double polyEval(JMTCoeffs_t,double);
    void createEndConstraint(double,double);
    void createStartConstraint();
    JMTCoeffs_t differentiate(JMTCoeffs_t);
 
    public: 
    TrajectoryJMT(std::shared_ptr<EgoVehicle> ep) : TrajectoryGeneration (ep){};
    virtual ~TrajectoryJMT(){};

    JMTCoeffs_t singleAxisJMT(SingleAxisState start, SingleAxisState end, double T);
    void calculateJMTPath(double pathDuration);

    protected:
    void generatePath() override; 
    
         
};

#endif