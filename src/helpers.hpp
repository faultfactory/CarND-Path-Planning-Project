#ifndef HELPERS
#define HELPERS
#include <math.h>
#include <vector>

constexpr double max_s = 6945.554;
constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }
int getLane(double d);
double distance(double x1, double y1, double x2, double y2);
double s_relative(double egoS, double tgtS);
int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y);
double getMapYaw(double s, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y);

#endif
