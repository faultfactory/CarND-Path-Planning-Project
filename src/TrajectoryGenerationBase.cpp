#include "TrajectoryGenerationBase.hpp"

void TrajectorySet::clear()
{
    xPts.clear();
    yPts.clear();
}

void TrajectorySet::concatenate(TrajectorySet tail)
{
    xPts.insert(xPts.end(), tail.xPts.begin(), tail.xPts.end());
    yPts.insert(yPts.end(), tail.yPts.begin(), tail.yPts.end());
}

void TrajectoryGeneration::initializeStubTrajectoryFromCurrent()
{
    refState.x = egoNow.x;
    refState.y = egoNow.y;
    refState.yaw = egoNow.yaw;
    refState.velocity = egoNow.v_mag;

    // Generate two points that align with current state;
    double prev_car_x = egoNow.x - cos(egoNow.yaw);
    double prev_car_y = egoNow.y - sin(egoNow.yaw);

    pathSeed.xPts.push_back(prev_car_x);
    pathSeed.xPts.push_back(egoNow.x);

    pathSeed.yPts.push_back(prev_car_y);
    pathSeed.yPts.push_back(egoNow.y);
};

void TrajectoryGeneration::setStubTrajectory()
{
    int prev_size = prior.previous_path_x.size();

    refState.x = prior.previous_path_x[prev_size - 1];
    refState.y = prior.previous_path_y[prev_size - 1];

    double vref_x = prior.previous_path_x[prev_size - 2];
    double vref_y = prior.previous_path_y[prev_size - 2];
    refState.velocity = (sqrt(pow((refState.x - vref_x), 2) + pow((refState.y - vref_y), 2))) / (systemCycleTime);

    double ref_x_prev;
    double ref_y_prev;

    // Guarantee monotonically increasing x values get added to spline.
    int setBack = 2;
    bool same = true;
    while (same == true)
    {
        same = (refState.x == prior.previous_path_x.at(prev_size - setBack));
        if (!same)
        {
            ref_x_prev = prior.previous_path_x.at(prev_size - setBack);
            ref_y_prev = prior.previous_path_y.at(prev_size - setBack);
        }
        else
        {
            setBack++;
        }
    }

    refState.yaw = atan2(refState.y - ref_y_prev, refState.x - ref_x_prev);

    pathSeed.xPts.push_back(ref_x_prev);
    pathSeed.xPts.push_back(refState.x);

    pathSeed.yPts.push_back(ref_y_prev);
    pathSeed.yPts.push_back(refState.y);
}

void TrajectoryGeneration::resetTrajectoryData()
{
    clearPathVectors();
    egoNow = egoPtr->getMostRecentFrame();
    prior = egoPtr->getPreviousPath();
    int prev_size = prior.previous_path_x.size();
    if (prev_size <= 2 || !priorPathValid)
    {
        initializeStubTrajectoryFromCurrent();
    }
    else
    {
        setStubTrajectory();
    }
};

TrajectorySet TrajectoryGeneration::transformToWorld(TrajectorySet vehicleFramePts)
{
    TrajectorySet worldFrameOut;
    int size = vehicleFramePts.xPts.size();

    for (int i = 0; i < size; i++)
    {
        double x_ref = vehicleFramePts.xPts.at(i);
        double y_ref = vehicleFramePts.yPts.at(i);
        double x_point = (x_ref * cos(refState.yaw) - y_ref * sin(refState.yaw));
        double y_point = (x_ref * sin(refState.yaw) + y_ref * cos(refState.yaw));
        x_point += refState.x;
        y_point += refState.y;

        worldFrameOut.xPts.push_back(x_point);
        worldFrameOut.yPts.push_back(y_point);
    }
    return worldFrameOut;
};

TrajectorySet TrajectoryGeneration::transformToVehicle(TrajectorySet worldFramePts)
{
    TrajectorySet vehicleFrameOut;
    int size = worldFramePts.xPts.size();

    for (int i = 0; i < size; i++)
    {
        double shift_x = worldFramePts.xPts.at(i) - refState.x;
        double shift_y = worldFramePts.yPts.at(i) - refState.y;
        vehicleFrameOut.xPts.push_back(shift_x * cos(0 - refState.yaw) - (shift_y)*sin(0 - refState.yaw));
        vehicleFrameOut.yPts.push_back(shift_x * sin(0 - refState.yaw) + (shift_y)*cos(0 - refState.yaw));
    }
    return vehicleFrameOut;
};

void TrajectoryGeneration::includePriorPathData()
{
    for (int i = 0; i < prior.previous_path_x.size(); i++)
    {
        outputPath.xPts.push_back(prior.previous_path_x.at(i));
        outputPath.yPts.push_back(prior.previous_path_y.at(i));
    }
};

TrajectorySet TrajectoryGeneration::generateNextPath()
{
    generatePath();
    return outputPath;
};

void TrajectoryGeneration::setTargetLane(int tL)
{
    targetLane = tL;
}

void TrajectoryGeneration::setTargetVelocity(double tV)
{
    targetVelocity = tV;
}
