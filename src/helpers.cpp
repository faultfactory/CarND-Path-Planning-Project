#include <iostream>
#include "helpers.hpp"
#include <math.h>
#include <vector>

using namespace std;



int getLane(double d)
{
    return  d/4;
}

double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}
int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{

	double closestLen = 100000; //large number
	int closestWaypoint = 0;

	for(int i = 0; i < maps_x.size(); i++)
	{
		double map_x = maps_x[i];
		double map_y = maps_y[i];
		double dist = distance(x,y,map_x,map_y);
		if(dist < closestLen)
		{
			closestLen = dist;
			closestWaypoint = i;
		}

	}

	return closestWaypoint;

}


double s_relative(double egoS, double tgtS)
{
	double diff = tgtS-egoS;
	if (diff>0 && diff<(max_s/2))
	{
		//car is 'more' ahead. show positive
		return diff;
	}
	else if(diff>0 && diff>(max_s/2))
	{
		// car is 'more' behind. show negative
		return diff-max_s;
	}
	else if(diff<0 && diff>(-max_s/2))
	{
		// car is more behind. show negative;
		return diff;
	}
	else if(diff<0 && diff<(-max_s/2))
	{
		// car is more ahead. show positive;
		return diff+max_s;
	}
}



// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
	int prev_wp = -1;

	while(s > maps_s[prev_wp+1] && (prev_wp < (int)(maps_s.size()-1) ))
	{
		prev_wp++;
	}

	int wp2 = (prev_wp+1)%maps_x.size();

	double heading = atan2((maps_y[wp2]-maps_y[prev_wp]),(maps_x[wp2]-maps_x[prev_wp]));
	// the x,y,s along the segment
	double seg_s = (s-maps_s[prev_wp]);

	double seg_x = maps_x[prev_wp]+seg_s*cos(heading);
	double seg_y = maps_y[prev_wp]+seg_s*sin(heading);

	double perp_heading = heading-pi()/2;

	double x = seg_x + d*cos(perp_heading);
	double y = seg_y + d*sin(perp_heading);

	return {x,y};

};

double getMapYaw(double s, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y)
{
    // returns the reference yaw value at the map for this location of s.
    std::vector<double> xy = getXY(s, 0.0, maps_s, maps_x, maps_y);
    int pt1 = ClosestWaypoint(xy.at(0), xy.at(1), maps_x, maps_y);
    int pt2 = pt1+1;
    if (pt2 == maps_x.size())
    {
        pt2 = 0;
    }
    // just leaving here for reference
//    x1=maps_x.at(pt1);
//    y1=maps_y.at(pt1);
//    x2=maps_x.at(pt2);
//    y2=maps_y.at(pt2);
    double roadYaw= atan2((maps_y.at(pt2)-maps_y.at(pt1)),(maps_x.at(pt2)-maps_x.at(pt1)));
    return roadYaw;
}

std::vector<double> getRoots(double a, double b, double c,double discriminant)
{
// referenced from: 
// https://www.programiz.com/cpp-programming/examples/quadratic-roots
	double x1, x2;
    std::vector<double> roots;
    
    if (discriminant > 0) {
        x1 = (-b + sqrt(discriminant)) / (2*a);
        x2 = (-b - sqrt(discriminant)) / (2*a);
    }
    
    else if (discriminant == 0) {
        x1 = (-b + sqrt(discriminant)) / (2*a);
        x2=x1;
    }
	roots.push_back(x1);roots.push_back(x2);
	return roots;
}

