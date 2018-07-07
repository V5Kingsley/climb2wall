#ifndef CLIMB_H_
#define CLIMB_H_

#include <cmath>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <cmath>

using std::vector;

class Climb_up
{
private: 
  vector<double> INIT_COXA_ANGLE;
  double BODY_RADIUS;
  double COXA_LENGTH, FEMUR_LENGTH, TIBIA_LENGTH, TARSUS_LENGTH;

 // vector<three_order_equation> coxa_interpolation, femur_interpolation, tibia_interpolation, tarsus_interpolation;

public:
  Climb_up();
  struct three_order_equation
  {
    double a0, a1, a2, a3;
  };
  struct inter_position
  {
    double position, speed;
  };
  bool climb_up_calculate(const int *leg_index, const double *distance, const double *theta2, double *theta1, double *theta3, double *theta4, double *feet_height);
  vector<three_order_equation> interpolation_equation( vector<inter_position>& position, double& tf );
  
};

#endif