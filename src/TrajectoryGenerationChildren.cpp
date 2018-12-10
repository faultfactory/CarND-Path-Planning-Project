#include "TrajectoryGenerationChildren.hpp"

void TrajectorySplineBased::setSpline()
{
    for (int i = 1; i < (waypoints + 1); i++)
    {
        // This does not work unless you add lane to the function line
        vector<double> next_wp = track->sd_to_xy(track->xy_to_sd(refState.x, refState.y).at(0) + (waypoint_s_increment * i), (2 + 4 * targetLane));
        pathSeed.xPts.push_back(next_wp.at(0));
        pathSeed.yPts.push_back(next_wp.at(1));
    }

    TrajectorySet vehicleFrame = transformToVehicle(pathSeed);
    spline.set_points(vehicleFrame.xPts, vehicleFrame.yPts);
}



void TrajectorySplineBased::generatePath()
{
    TrajectorySet vehicleFramePath;
    double target_x = 30.0;
    double target_y = spline(target_x);
    double target_dist = sqrt((target_x * target_x) + (target_y * target_y));
    // TODO: Since we're not inside the loop for this calculation, there's a loss of accuracy.
    // Consider Recalculating dist information on an incremental basis.
    double x_cumulative = 0.0;
    if (priorPathValid)
    {
        includePriorPathData();
    }
    else
    {
        pathReferenceVelocity = refState.velocity;
    }

    for (int i = 0; i <= pathCount - outputPath.xPts.size(); i++)
    {
        updateReferenceVelocity();
        double N = (target_dist / (systemCycleTime * pathReferenceVelocity));
        double x_point = x_cumulative + (target_x) / N;
        double y_point = spline(x_point);

        x_cumulative = x_point;

        vehicleFramePath.xPts.push_back(x_point);
        vehicleFramePath.yPts.push_back(y_point);
    }

    outputPath.concatenate(transformToWorld(vehicleFramePath));
}


void TrajectorySplineBased::updateReferenceVelocity() 
{

    double veldiff = pathReferenceVelocity - targetVelocity;
    bool change = fabs(veldiff) > vel_inc;

    if (change)
    {
        if (veldiff < 0.0)
        {
            pathReferenceVelocity += vel_inc;
        }
        else if (veldiff > 0.0)
        {
            pathReferenceVelocity -= vel_inc;
        }
    }
    else
    {
        pathReferenceVelocity = targetVelocity;
    }
}


 
// Produces polynomial coefficients for trajectory
JMTCoeffs_t TrajectoryJMT::singleAxisJMT(SingleAxisState start, SingleAxisState end, double T)
{
    //TODO: Dig into Eigen documentation for built-in methods to make this cleaner
    // (Not advanced initialization)
    Eigen::MatrixXd A = Eigen::MatrixXd(3, 3);
    A.row(0) = Eigen::Vector3d(T * T * T, T * T * T * T, T * T * T * T);
    A.row(1) = Eigen::Vector3d(3 * T * T, 4 * T * T * T, 5 * T * T * T * T);
    A.row(2) = Eigen::Vector3d(6 * T, 12 * T * T, 20 * T * T * T);

    Eigen::MatrixXd B = Eigen::MatrixXd(3, 1);
    B << end.pos - (start.pos + start.vel * T + .5 * start.acc * T * T),
        end.vel - (start.vel + start.acc * T),
        end.acc - start.acc;

    auto C = A.inverse() * B;

    vector<double> result = {start.pos, start.vel, .5 * start.acc};
    for (int i = 0; i < C.size(); i++)
    {
        result.push_back(C.data()[i]);
    }

    return result;
}

double polyEval(JMTCoeffs_t coeffs, double t)
{
    double output = 0.0;

    double t_term = 1;
    for (auto coeff : coeffs)
    {
        output += coeff * t_term;
        t_term *= t;
    }
    return output;
}

JMTCoeffs_t TrajectoryJMT::differentiate(JMTCoeffs_t input)
{
    JMTCoeffs_t derivative;
    int i = 0;
    for (auto coeff : input)
    {
        double newTerm = double(i) * coeff;

        if (i > 0)
        {
            derivative.push_back(newTerm);
        }
    }
    return derivative;
}

void TrajectoryJMT::createStartConstraint()
{
    start.x.pos = refState.x;
    start.y.pos = refState.y;

    start.x.vel = refState.velocity * cos(refState.yaw);
    start.y.vel = refState.velocity * sin(refState.yaw);

    if (prior.previous_path_x.empty())
    {
        start.x.acc = 0.0;
        start.y.acc = 0.0;
    }
    else
    {
        JMTCoeffs_t xAccelerationFunction = differentiate(differentiate(computedPath.x));
        JMTCoeffs_t yAccelerationFunction = differentiate(differentiate(computedPath.y));

        start.x.acc = polyEval(xAccelerationFunction, consumedIncrements * systemCycleTime);
        start.y.acc = polyEval(yAccelerationFunction, consumedIncrements * systemCycleTime);
    }
}

void TrajectoryJMT::createEndConstraint(double endpointDistance, double endpointSVelocity)
{
    double endSPos = track->xy_to_sd(refState.x, refState.y).at(0) + endpointDistance;
    double endDPos = (2 + 4 * targetLane);

    vector<double> endpointTargets = track->sd_to_xyv(endSPos, endDPos, targetVelocity, 0.0);
    end.x.pos = endpointTargets.at(0);
    end.y.pos = endpointTargets.at(1);
    end.x.vel = endpointTargets.at(2);
    end.y.vel = endpointTargets.at(3);
    end.x.acc = 0.0;
    end.y.acc = 0.0;
}

void TrajectoryJMT::getNewPathCount()
{
    if (!priorPathValid)
    {
        consumedIncrements = pathCount - prior.previous_path_x.size();
    }
    else
    {
        consumedIncrements = pathCount;
    }
}

void TrajectoryJMT::calculateJMTPath(double pathDuration)
{
    computedPath.x = singleAxisJMT(start.x, end.x, pathDuration);
    computedPath.y = singleAxisJMT(start.y, end.y, pathDuration);
}

void TrajectoryJMT::generatePath()
{
    if (priorPathValid)
    {
        includePriorPathData();
    }

    getNewPathCount();
    createStartConstraint();
    createEndConstraint();
    calculateJMTPath();





    for (int i = 0; i <= pathCount - outputPath.xPts.size(); i++)
    {
    }
}


