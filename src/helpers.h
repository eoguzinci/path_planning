#ifndef HELPERS_H
#define HELPERS_H

#include <iostream>
#include <math.h>
#include <vector>
#include <numeric>
#include <<cfloat>>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "spline.h"

using namespace std;

int ClosestWaypoint(double x, double y, vector<double> maps_x, vector<double> maps_y);
int NextWaypoint(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);

constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);
double distance(double x1, double y1, double x2, double y2);
vector<double> getFrenet(double x, double y, double theta, vector<double> maps_x, vector<double> maps_y);
vector<double> getXY(double s, double d, vector<double> maps_s, vector<double> maps_x, vector<double> maps_y);

template<typename Iter_T>
long double vectorNorm(Iter_T first, Iter_T last) {
  return sqrt(inner_product(first, last, first, 0.0L));
}

double mph2ms(double);
double ms2mph(double);

#endif //PATH_PLANNING_HELPER_FUNCTIONS_H
