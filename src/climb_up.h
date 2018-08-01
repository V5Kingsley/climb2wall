#ifndef CLIMB_H_
#define CLIMB_H_

#include <cmath>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <cmath>
#include <std_msgs/Float64.h>

using std::vector;

class Climb_up
{
private: 
  vector<double> INIT_COXA_ANGLE;
  double BODY_RADIUS;
  double COXA_LENGTH, FEMUR_LENGTH, TIBIA_LENGTH, TARSUS_LENGTH;
  
  ros::NodeHandle nh_;
  //关节话题
  std::string leg_topic[24]; 
  ros::Publisher leg_coxa_p[6];
  ros::Publisher leg_femur_p[6];
  ros::Publisher leg_tibia_p[6];
  ros::Publisher leg_tarsus_p[6];
  std_msgs::Float64 leg_coxa[6];
  std_msgs::Float64 leg_femur[6];
  std_msgs::Float64 leg_tibia[6];
  std_msgs::Float64 leg_tarsus[6];
  

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
  vector< vector<three_order_equation> > equation_coe;
  bool climb_up_calculate(const int *leg_index, const double *distance, const double *theta2, double *theta1, double *theta3, double *theta4, double *feet_height);
  void interpolation_equation(int joint_num,  vector< vector<Climb_up::inter_position> >& position,  double &tf);
  void coe_calculate( vector<inter_position>& position, const double& tf,  vector<three_order_equation>& equation_coe);
  void clear_equation_coe();
  void publish_interprolation_joint_states(const int& leg_index, vector< vector<three_order_equation> >& equation_coe, const double& tf );
  void publish_interprolation_all_joint_states(vector< vector<three_order_equation> >& equation_coe, const double& tf);
  void roll_translation_left(const int leg_index, const double* theta2, const double theta2_0, const double roll_0, const double roll_t, const double translation, double* theta1, double* theta3, double* theta4, double theta4_0);
  void roll_translation_right(const int leg_index, const double* theta2, const double theta2_0, const double roll_0, const double roll_t, const double translation, double* theta1, double* theta3, double* theta4, double theta4_0);
  
  void roll_translation(const int& leg_index, double* joint, const double& roll_t, const double& translation, double& roll_0, const double& theta3_0, const double& theta4_0);
  
};

#endif