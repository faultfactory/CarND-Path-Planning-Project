#include "TrajectoryGeneration.hpp"

void TrajectorySet::clear()
{
    xPts.clear();
    yPts.clear();
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

    double ref_x_prev = prior.previous_path_x[prev_size - 2];
    double ref_y_prev = prior.previous_path_y[prev_size - 2];
    refState.yaw = atan2(refState.y - ref_y_prev, refState.x - ref_x_prev);
    refState.velocity = sqrt(pow((refState.x - ref_x_prev), 2) + pow((refState.y - ref_y_prev), 2));

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
    if (prev_size < 2)
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
}

void TrajectorySplineBased::setSpline()
{
    for (int i = 1; i < (waypoints + 1); i++)
    {
        // This does not work unless you add lane to the function line
        vector<double> next_wp = track->sd_to_xy(track->xy_to_sd(refState.x, refState.y).at(0) + (waypoint_s_increment * i), (2 + 4 * targetLane));
        pathSeed.xPts.push_back(next_wp.at(0));
        pathSeed.yPts.push_back(next_wp.at(1));
    }
}

void TrajectorySplineBased::updateReferenceVelocity()
{
    double veldiff = pathReferenceVelocity - targetVelocity;
    bool change = fabs(veldiff) > vel_inc;

    if (change)
    {
        if (veldiff < vel_inc)
        {
            pathReferenceVelocity += vel_inc;
        }
        else if (veldiff > vel_inc)
        {
            pathReferenceVelocity -= vel_inc;
        }
    }
    else
    {
        pathReferenceVelocity = targetVelocity;
    }
};

void TrajectorySplineBased::generatePath()
{
    TrajectorySet vehicleFramePath;
    double target_x = 30.0;
    double target_y = (target_x);
    double target_dist = sqrt((target_x * target_x) + (target_y * target_y));
    // TODO: Since we're not inside the loop for this calculation, there's a loss of accuracy. 
    // Consider Recalculating dist information on an incremental basis. 

    double x_cumulative = 0.0;
    for (int i = 1; i <= pathCount - outputPath.xPts.size(); i++)
    {

        double N = (target_dist / (0.02 * pathReferenceVelocity));
        double x_point = x_cumulative + (target_x) / N;
        double y_point = spline(x_point);

        x_cumulative = x_point;

        double x_ref = x_point;
        double y_ref = y_point;

        //transform system back to world coordinates after defining path spline

        ////// Transform2Vehicle

        x_point = (x_ref * cos(refState.yaw) - y_ref * sin(refState.yaw));
        y_point = (x_ref * sin(refState.yaw) + y_ref * cos(refState.yaw));

        x_point += refState.x;
        y_point += refState.y;

        vehicleFramePath.xPts.push_back(x_point);
        vehicleFramePath.yPts.push_back(y_point);
    }
}

void TrajectorySplineBased::setTargetLane(int tL)
{
    targetLane = tL;
}

void TrajectorySplineBased::setTargetVelocity(double tV)
{
    targetVelocity = tV;
}