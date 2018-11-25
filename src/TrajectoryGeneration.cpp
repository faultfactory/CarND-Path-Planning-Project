#include "TrajectoryGeneration.hpp"

void TrajectoryGeneration::initializeStubTrajectory()
{
    ref_position.x = egoNow.x;
    ref_position.y = egoNow.y;
    ref_position.yaw = egoNow.yaw;

    // Generate two points that align with current state;
    double prev_car_x = egoNow.x - cos(egoNow.yaw);
    double prev_car_y = egoNow.y - sin(egoNow.yaw);

    output.xPts.push_back(prev_car_x);
    output.xPts.push_back(egoNow.x);

    output.yPts.push_back(prev_car_y);
    output.yPts.push_back(egoNow.y);
};

void TrajectoryGeneration::setStubTrajectory(PathStatus *prior)
{
    int prev_size = prior->previous_path_x.size();
    ref_position.x = prior->previous_path_x[prev_size - 1];
    ref_position.y = prior->previous_path_y[prev_size - 1];

    double ref_x_prev = prior->previous_path_x[prev_size - 2];
    double ref_y_prev = prior->previous_path_y[prev_size - 2];
    ref_position.yaw = atan2(ref_position.y - ref_y_prev, ref_position.x - ref_x_prev);

    output.xPts.push_back(ref_x_prev);
    output.xPts.push_back(ref_position.x);

    output.yPts.push_back(ref_y_prev);
    output.yPts.push_back(ref_position.y);
}

void TrajectoryGeneration::resetTrajectoryData()
{
    clearPathVectors();
    egoNow = egoPtr->getMostRecentFrame();
    prior = egoPtr->getPreviousPath();
    int prev_size = prior.previous_path_x.size();
    if (prev_size < 2)
    {
        initializeStubTrajectory();
    }
    else
    {
        setStubTrajectory(&prior);
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
        double x_point = (x_ref * cos(ref_position.yaw) - y_ref * sin(ref_position.yaw));
        double y_point = (x_ref * sin(ref_position.yaw) + y_ref * cos(ref_position.yaw));
        x_point += ref_position.x;
        y_point += ref_position.y;

        worldFrameOut.xPts.push_back(x_point);
        worldFrameOut.yPts.push_back(y_point);
    }
    return worldFrameOut;
};

TrajectorySet TrajectoryGeneration::transformToVehicle(TrajectorySet worldFramePts)
{
    TrajectorySet vehicleFrameOut;
    int size = worldFramePts.xPts.size();

    TrajectorySet safe;

    double shift_x = worldFramePts.xPts[0] - ref_position.x;
    double shift_y = worldFramePts.yPts[0] - ref_position.y;

    safe.xPts.push_back(shift_x * cos(0 - ref_position.yaw) - (shift_y)*sin(0 - ref_position.yaw));
    safe.yPts.push_back(shift_x * sin(0 - ref_position.yaw) + (shift_y)*cos(0 - ref_position.yaw));

    for (int i = 0; i < size; i++)
    {

        double shift_x = worldFramePts.xPts[i] - ref_position.x;
        double shift_y = worldFramePts.yPts[i] - ref_position.y;
        double testx = shift_x * cos(0 - ref_position.yaw) - (shift_y)*sin(0 - ref_position.yaw);
        // Test for duplicates and if not a duplicate, add to set.
        // TODO: When points are converted to Eigen consider a 'safeAdd' or use a container with dupe checking.
        if (safe.xPts[i - 1] != testx)
        {
            safe.xPts.push_back(shift_x * cos(0 - ref_position.yaw) - (shift_y)*sin(0 - ref_position.yaw));
            safe.yPts.push_back(shift_x * sin(0 - ref_position.yaw) + (shift_y)*cos(0 - ref_position.yaw));
        }
    }
    return safe;
};