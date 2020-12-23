#include "math.h"

double deg2rad(double x) {
  return (x * M_PI) / 180;
}
double rad2deg(double x) {
  return (x * 180) / M_PI;
}
double distance(double x1, double y1, double x2, double y2) {
  return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}
double MPH2mps(double speed){
    return speed * 0.44704;
}
double wrappedDistance(double back_s, double front_s) {
  double distance = (front_s - back_s + 6945.554) - 6945.554;

  if (distance < 0) {
    distance = 6945.554 + distance;
  }
  return distance;
}