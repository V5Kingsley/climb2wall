#include "climb_up.h"

Climb_up::Climb_up()
{
  ros::param::get( "INIT_COXA_ANGLE", INIT_COXA_ANGLE );
  ros::param::get("BODY_RADIUS", BODY_RADIUS);
  ros::param::get( "COXA_LENGTH", COXA_LENGTH );
  ros::param::get( "FEMUR_LENGTH", FEMUR_LENGTH );
  ros::param::get( "TIBIA_LENGTH", TIBIA_LENGTH );
  ros::param::get( "TARSUS_LENGTH", TARSUS_LENGTH );
}

//计算六足离墙体distance，股关节抬起theta2角度，垂直贴在墙面时的关节角
bool Climb_up::climb_up_calculate(const int *leg_index, const double *distance, const double *theta2, double *theta1, double *theta3, double *theta4, double *feet_height)
{
  if( *distance > (BODY_RADIUS + COXA_LENGTH + FEMUR_LENGTH) && *distance < (0.5*BODY_RADIUS + COXA_LENGTH + FEMUR_LENGTH + TIBIA_LENGTH + TARSUS_LENGTH) )
  {
    double femur2foot, femur2tarsus;
    femur2foot = *distance - BODY_RADIUS * cos(INIT_COXA_ANGLE[*leg_index]) - COXA_LENGTH; //d1
    femur2tarsus = femur2foot - TARSUS_LENGTH; //d2
    *theta3 = std::abs(asin ( ( femur2tarsus - FEMUR_LENGTH * cos(*theta2) ) / TIBIA_LENGTH ) - *theta2);
    *theta4 = M_PI/2 - *theta2 - *theta3;
    *theta1 = - INIT_COXA_ANGLE[*leg_index];
    *feet_height = FEMUR_LENGTH * sin(*theta2) - TIBIA_LENGTH * sin(*theta2 + *theta3) + TIBIA_LENGTH + TARSUS_LENGTH;
    return true;
  }
  else
  {
    ROS_FATAL("The hexapod is out of safe distance!!!");
    return false;
  }
 
}

//关节空间三次插值，返回插值的三次函数系数值
vector< Climb_up::three_order_equation > Climb_up::interpolation_equation( vector< Climb_up::inter_position >& position,  double &tf)
{
  vector<three_order_equation> equation_coe;
  vector<Climb_up::inter_position>::iterator pd;
  for(pd = position.begin(); pd != (position.end()-1); pd++)
  {
    three_order_equation equ;
    equ.a0 = pd->position;
    equ.a1 = pd->speed;
    equ.a2 = 3.0 / pow(tf, 2) * ((pd+1)->position - pd->position) - 2 / tf * pd->speed - 1 / tf * ((pd+1)->speed);
    equ.a3 = -2 / pow(tf, 3) * ((pd+1)->position - pd->position) + 1 / pow(tf, 2) * (pd->speed + (pd+1)->speed);
    equation_coe.push_back(equ);
  }
  return equation_coe;
  
}



