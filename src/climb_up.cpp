/************************************************
 *        爬墙算法程序                           *
 *        Copyright (c) V5_Lab, 2018            *
 *        Author :          Andy, Kingsley      *
 *        Version number :  0.0                 *
 *        Date :                                *
 ************************************************/

#include "climb_up.h"

Climb_up::Climb_up()
{
  ros::param::get( "INIT_COXA_ANGLE", INIT_COXA_ANGLE );
  ros::param::get("BODY_RADIUS", BODY_RADIUS);
  ros::param::get( "COXA_LENGTH", COXA_LENGTH );
  ros::param::get( "FEMUR_LENGTH", FEMUR_LENGTH );
  ros::param::get( "TIBIA_LENGTH", TIBIA_LENGTH );
  ros::param::get( "TARSUS_LENGTH", TARSUS_LENGTH );
  
  
  //发布的关节角度话题
    boost::format coxa;
    boost::format femur;
    boost::format tibia;
    boost::format tarsus;
    for ( int leg_index = 0,  j =1;  leg_index < 6;  leg_index ++, j ++ )
    {
       coxa = boost::format("/hexapod/leg%d_roll_joint_position_controller/command") % j;
       femur = boost::format("/hexapod/leg%d_pitch1_joint_position_controller/command") % j;
       tibia = boost::format("/hexapod/leg%d_pitch2_joint_position_controller/command") % j; 
       tarsus = boost::format("/hexapod/leg%d_pitch3_joint_position_controller/command") % j;
      leg_topic[leg_index] = coxa.str();
      leg_topic[leg_index+1] = femur.str();
      leg_topic[leg_index+2] = tibia.str();
      leg_topic[leg_index+3] = tarsus.str();
      leg_coxa_p[leg_index] = nh_.advertise<std_msgs::Float64>( leg_topic[leg_index], 10 );
      leg_femur_p[leg_index] = nh_.advertise<std_msgs::Float64>( leg_topic[leg_index+1], 10 );
      leg_tibia_p[leg_index] = nh_.advertise<std_msgs::Float64>( leg_topic[leg_index+2], 10 );
      leg_tarsus_p[leg_index] = nh_.advertise<std_msgs::Float64>( leg_topic[leg_index+3], 10 );
    }
}

//计算六足离墙体distances，股关节抬起theta2角度时，吸盘垂直贴在墙面时的关节角
bool Climb_up::climb_up_calculate(const int *leg_index, const double *distance, const double *theta2, double *theta1, double *theta3, double *theta4, double *feet_height)
{
  if( *distance > (BODY_RADIUS + COXA_LENGTH + FEMUR_LENGTH) && *distance < (0.5*BODY_RADIUS + COXA_LENGTH + FEMUR_LENGTH + TIBIA_LENGTH + TARSUS_LENGTH) )
  {
    double femur2foot, femur2tarsus;
    femur2foot = *distance - BODY_RADIUS * sin(INIT_COXA_ANGLE[*leg_index]) - COXA_LENGTH; //d1
    femur2tarsus = femur2foot - TARSUS_LENGTH; //d2
    
    if ( (femur2tarsus-FEMUR_LENGTH*cos(*theta2)+TARSUS_LENGTH)<=(TIBIA_LENGTH+TARSUS_LENGTH) )
    {
//        *theta3 = M_PI - asin ( ( femur2tarsus - FEMUR_LENGTH * cos(*theta2) ) / TIBIA_LENGTH ) - *theta2;
//       *theta4 = M_PI/2 - *theta2 - *theta3;
      *theta4 = - acos( (femur2tarsus - FEMUR_LENGTH*cos(*theta2)) / TIBIA_LENGTH );
      *theta3 = M_PI/2 - *theta2 - *theta4;
      *theta1 = M_PI/2 - INIT_COXA_ANGLE[*leg_index];
      *feet_height = FEMUR_LENGTH * sin(*theta2) - TIBIA_LENGTH * sin(*theta2 + *theta3) + TIBIA_LENGTH + TARSUS_LENGTH;
      return true;
    }
    else
    {
      ROS_FATAL("11The hexapod is out of safe distance!!!");
      return false;
    }
    
  }
  else
  {
    ROS_FATAL("The hexapod is out of safe distance!!!");
    return false;
  }
 
}

//关节空间三次插值，计算插值的每个三次函数系数值
void Climb_up::interpolation_equation(int joint_num, vector< vector<Climb_up::inter_position> >& position,  double& tf)
{
  equation_coe.resize(joint_num);
  for(int i = 0; i < joint_num; i++)
  {
    coe_calculate(position[i], tf, equation_coe[i]);
  }
}

//单个三次函数系数值计算
void Climb_up::coe_calculate( vector< Climb_up::inter_position >& position, const double& tf, vector< Climb_up::three_order_equation >& equation_coe)
{
  vector<inter_position>::iterator pd;
  for(pd = position.begin(); pd != (position.end()-1); pd++)
  {
    three_order_equation equ;
    equ.a0 = pd->position;
    equ.a1 = pd->speed;
    equ.a2 = 3.0 / pow(tf, 2) * ((pd+1)->position - pd->position) - 2 / tf * pd->speed - 1 / tf * ((pd+1)->speed);
    equ.a3 = -2 / pow(tf, 3) * ((pd+1)->position - pd->position) + 1 / pow(tf, 2) * (pd->speed + (pd+1)->speed);
    equation_coe.push_back(equ);
  }
}

//清除三次函数系数值，回收空间，每次重新插值前都需调用
void Climb_up::clear_equation_coe()
{
  vector < vector<three_order_equation> > ().swap(equation_coe);
}

//插值后利用插值函数发布单条腿角度话题
void Climb_up::publish_interprolation_joint_states(const int& leg_index, vector< vector< Climb_up::three_order_equation > >& equation_coe, const double& tf)
{
  int cycle_length = 50.0; //插值长度
  
  vector< Climb_up::three_order_equation >::iterator coxa_pd = equation_coe[0].begin();
  vector< Climb_up::three_order_equation >::iterator femur_pd = equation_coe[1].begin();
  vector< Climb_up::three_order_equation >::iterator tibia_pd = equation_coe[2].begin();
  vector< Climb_up::three_order_equation >::iterator tarsus_pd = equation_coe[3].begin();
  
  //用每个关节的插值函数分别计算关节角
  for( coxa_pd = equation_coe[0].begin(); coxa_pd != equation_coe[0].end(); coxa_pd++, femur_pd++, tibia_pd++, tarsus_pd++ )
  {
    for(int i = 0; i < cycle_length; i++)
    {
      leg_coxa[leg_index].data = coxa_pd->a0 + coxa_pd->a1 * (tf*i/cycle_length) + coxa_pd->a2 * pow((tf*i/cycle_length), 2) + coxa_pd->a3 * pow((tf*i/cycle_length), 3);
      leg_femur[leg_index].data = femur_pd->a0 + femur_pd->a1 * (tf*i/cycle_length) + femur_pd->a2 * pow((tf*i/cycle_length), 2) + femur_pd->a3 * pow((tf*i/cycle_length), 3);
      leg_tibia[leg_index].data = tibia_pd->a0 + tibia_pd->a1 * (tf*i/cycle_length) + tibia_pd->a2 * pow((tf*i/cycle_length), 2) + tibia_pd->a3 * pow((tf*i/cycle_length), 3);
      leg_tarsus[leg_index].data = tarsus_pd->a0 + tarsus_pd->a1 * (tf*i/cycle_length) + tarsus_pd->a2 * pow((tf*i/cycle_length), 2) + tarsus_pd->a3 * pow((tf*i/cycle_length), 3);
      leg_coxa_p[leg_index].publish(leg_coxa[leg_index]);
      leg_femur_p[leg_index].publish(leg_femur[leg_index]);
      leg_tibia_p[leg_index].publish(leg_tibia[leg_index]);
      leg_tarsus_p[leg_index].publish(leg_tarsus[leg_index]);
      ros::Duration(0.02).sleep();
    }
  }
  
  //最后一个插值函数的终值补偿
  coxa_pd = equation_coe[0].end() - 1;
  femur_pd = equation_coe[1].end() - 1;
  tibia_pd = equation_coe[2].end() - 1;
  tarsus_pd = equation_coe[3].end() - 1;
  leg_coxa[leg_index].data = coxa_pd->a0 + coxa_pd->a1 * tf + coxa_pd->a2 * pow(tf, 2) + coxa_pd->a3 * pow(tf, 3);
  leg_femur[leg_index].data = femur_pd->a0 + femur_pd->a1 *tf + femur_pd->a2 * pow(tf, 2) + femur_pd->a3 * pow(tf, 3);
  leg_tibia[leg_index].data = tibia_pd->a0 + tibia_pd->a1 * tf + tibia_pd->a2 * pow(tf, 2) + tibia_pd->a3 * pow(tf, 3);
  leg_tarsus[leg_index].data = tarsus_pd->a0 + tarsus_pd->a1 * tf + tarsus_pd->a2 * pow(tf, 2) + tarsus_pd->a3 * pow(tf, 3);
  leg_coxa_p[leg_index].publish(leg_coxa[leg_index]);
  leg_femur_p[leg_index].publish(leg_femur[leg_index]);
  leg_tibia_p[leg_index].publish(leg_tibia[leg_index]);
  leg_tarsus_p[leg_index].publish(leg_tarsus[leg_index]);
}


//插值后利用插值函数发布所有腿的关节话题
void Climb_up::publish_interprolation_all_joint_states(vector< vector< Climb_up::three_order_equation > >& equation_coe, const double& tf)
{
  int cycle_length = 50.0;  //插值长度
  vector< Climb_up::three_order_equation >::iterator coxa_pd[6]; 
  vector< Climb_up::three_order_equation >::iterator femur_pd[6];
  vector< Climb_up::three_order_equation >::iterator tibia_pd[6];
  vector< Climb_up::three_order_equation >::iterator tarsus_pd[6];
  
  //初始化每个迭代器，指向对应关节角的第一个插值函数地址
  for(int i = 0; i < 6; i++)
  {
    coxa_pd[i] = equation_coe[4*i].begin();
    femur_pd[i] = equation_coe[4*i+1].begin();
    tibia_pd[i] = equation_coe[4*i+2].begin();
    tarsus_pd[i] = equation_coe[4*i+3].begin();
  }
  
  //用每个关节的插值函数分别计算关节角
  for(coxa_pd[0] = equation_coe[0].begin(); coxa_pd[0] != equation_coe[0].end(); coxa_pd[0]++)
  {
    for(int i = 0; i < cycle_length; i++)
    {
      for(int leg_index = 0; leg_index < 6; leg_index++)
      {
	leg_coxa[leg_index].data = coxa_pd[leg_index]->a0 + coxa_pd[leg_index]->a1 * (tf*i/cycle_length) + coxa_pd[leg_index]->a2 * pow((tf*i/cycle_length), 2) + coxa_pd[leg_index]->a3 * pow((tf*i/cycle_length), 3);
	leg_femur[leg_index].data = femur_pd[leg_index]->a0 + femur_pd[leg_index]->a1 * (tf*i/cycle_length) + femur_pd[leg_index]->a2 * pow((tf*i/cycle_length), 2) + femur_pd[leg_index]->a3 * pow((tf*i/cycle_length), 3);
        leg_tibia[leg_index].data = tibia_pd[leg_index]->a0 + tibia_pd[leg_index]->a1 * (tf*i/cycle_length) + tibia_pd[leg_index]->a2 * pow((tf*i/cycle_length), 2) + tibia_pd[leg_index]->a3 * pow((tf*i/cycle_length), 3);
        leg_tarsus[leg_index].data = tarsus_pd[leg_index]->a0 + tarsus_pd[leg_index]->a1 * (tf*i/cycle_length) + tarsus_pd[leg_index]->a2 * pow((tf*i/cycle_length), 2) + tarsus_pd[leg_index]->a3 * pow((tf*i/cycle_length), 3);
        leg_coxa_p[leg_index].publish(leg_coxa[leg_index]);
        leg_femur_p[leg_index].publish(leg_femur[leg_index]);
       leg_tibia_p[leg_index].publish(leg_tibia[leg_index]);
        leg_tarsus_p[leg_index].publish(leg_tarsus[leg_index]);
      }
      ros::Duration(0.02).sleep();
    }
    for(int j = 0; j < 6; j++)
    {
      coxa_pd[j]++;
      femur_pd[j]++;
      tibia_pd[j]++;
      tarsus_pd[j]++;
    }
    coxa_pd[0]--;
  }
  
  //最后一个插值函数的终值补偿
   for(int i = 0; i < 6; i++)
    {
      coxa_pd[i] = equation_coe[4*i].end() - 1;
      femur_pd[i] = equation_coe[4*i+1].end() - 1;
      tibia_pd[i] = equation_coe[4*i+2].end() - 1;
      tarsus_pd[i] = equation_coe[4*i+3].end() - 1;
    }
    for(int leg_index = 0; leg_index < 6; leg_index++)
    {
      leg_coxa[leg_index].data = coxa_pd[leg_index]->a0 + coxa_pd[leg_index]->a1 * tf + coxa_pd[leg_index]->a2 * pow(tf, 2) + coxa_pd[leg_index]->a3 * pow(tf, 3);
      leg_femur[leg_index].data = femur_pd[leg_index]->a0 + femur_pd[leg_index]->a1 *tf + femur_pd[leg_index]->a2 * pow(tf, 2) + femur_pd[leg_index]->a3 * pow(tf, 3);
      leg_tibia[leg_index].data = tibia_pd[leg_index]->a0 + tibia_pd[leg_index]->a1 * tf + tibia_pd[leg_index]->a2 * pow(tf, 2) + tibia_pd[leg_index]->a3 * pow(tf, 3);
      leg_tarsus[leg_index].data = tarsus_pd[leg_index]->a0 + tarsus_pd[leg_index]->a1 * tf + tarsus_pd[leg_index]->a2 * pow(tf, 2) + tarsus_pd[leg_index]->a3 * pow(tf, 3);
      leg_coxa_p[leg_index].publish(leg_coxa[leg_index]);
      leg_femur_p[leg_index].publish(leg_femur[leg_index]);
      leg_tibia_p[leg_index].publish(leg_tibia[leg_index]);
      leg_tarsus_p[leg_index].publish(leg_tarsus[leg_index]);
    }
  
}





//左三腿机身俯仰(roll_t - roll_0)角度， 向y轴平移translation，根据输入的theta2初始值和最终值以及theta4初始值，得出theta1, theta3, theta4
void Climb_up::roll_translation_left(const int leg_index, const double* theta2, const double theta2_0, const double roll_0, const double roll_t, const double translation, double* theta1, double* theta3, double* theta4, double theta4_0)
{
   double num1 = translation * sin(roll_t);
  double num2 = ( COXA_LENGTH + BODY_RADIUS * sin(INIT_COXA_ANGLE[leg_index]) ) * ( sin(roll_t) - sin(roll_0) );
  double num3 = FEMUR_LENGTH * ( sin(roll_t+*theta2) - sin(roll_0+theta2_0) );
  double num4 = TIBIA_LENGTH * sin(theta4_0);
  double num = num1 + num2 + num3 + num4;
  
  double den1 = - translation * cos(roll_t);
  double den2 = - ( COXA_LENGTH + BODY_RADIUS * sin(INIT_COXA_ANGLE[leg_index]) ) * ( cos(roll_t) - cos(roll_0) );
  double den3 = - FEMUR_LENGTH * ( cos(roll_t+*theta2) - cos(roll_0+theta2_0) );
  double den4 = TIBIA_LENGTH * cos(theta4_0);
  double den = den1 + den2 + den3 + den4;
  
  *theta4 = atan(num/den) - M_PI/2;
  *theta1 = M_PI/2 - INIT_COXA_ANGLE[leg_index];
  *theta3 = std::abs(M_PI/2 - roll_t - *theta2 - *theta4);
 
/*  double num1 = translation * cos(roll_t);
  double num2 = ( COXA_LENGTH + (BODY_RADIUS) * sin(INIT_COXA_ANGLE[leg_index]) ) * ( cos(roll_t) - cos(roll_0) );
  double num3 = FEMUR_LENGTH * ( cos(roll_t+*theta2) - cos(roll_0+theta2_0) );
  double num4 = TIBIA_LENGTH * sin(theta4_0);
  double num = num1 + num2 + num3 + num4;
  
  double den1 = - translation * (-sin(roll_t));
  double den2 =  ( COXA_LENGTH + (BODY_RADIUS) * sin(INIT_COXA_ANGLE[leg_index]) ) * ( sin(roll_t) - sin(roll_0) );
  double den3 =  FEMUR_LENGTH * ( sin(roll_t+*theta2) - sin(roll_0+theta2_0) );
  double den4 = TIBIA_LENGTH * cos(theta4_0);
  double den = den1 + den2 + den3 + den4;
  
  *theta4 = atan(num/den)  ;
  *theta3 = -roll_t - *theta2 - *theta4;
  *theta1 = M_PI/2 - INIT_COXA_ANGLE[leg_index]; */
}

//右三腿机身俯仰(roll_t - roll_0)角度， 向y轴平移translation，根据输入的theta2初始值和最终值以及theta4初始值，得出theta1, theta3, theta4
void Climb_up::roll_translation_right(const int leg_index, const double* theta2, const double theta2_0, const double roll_0, const double roll_t, const double translation, double* theta1, double* theta3, double* theta4, double theta4_0)
{
  double num1 = - translation * cos(roll_t);
  double num2 = ( COXA_LENGTH - (BODY_RADIUS) * sin(INIT_COXA_ANGLE[leg_index]) ) * ( cos(roll_t) - cos(roll_0) );
  double num3 = FEMUR_LENGTH * ( cos(roll_t-*theta2) - cos(roll_0-theta2_0) );
  double num4 = TIBIA_LENGTH * sin(theta4_0);
  double num = num1 + num2 + num3 + num4;
  
  double den1 = translation * sin(roll_t);
  double den2 = - ( COXA_LENGTH - (BODY_RADIUS) * sin(INIT_COXA_ANGLE[leg_index]) ) * ( sin(roll_t) - sin(roll_0) );
  double den3 = - FEMUR_LENGTH * ( sin(roll_t-*theta2) - sin(roll_0-theta2_0) );
  double den4 = TIBIA_LENGTH * cos(theta4_0);
  double den = den1 + den2 + den3 + den4;
  
  *theta4 =   atan(num/den);
  *theta1 = - M_PI/2 - INIT_COXA_ANGLE[leg_index];
  *theta3 = roll_t - *theta2 - *theta4;
}

//机身俯仰平移version 2018.7.31
void Climb_up::roll_translation(const int& leg_index, double* joint, const double& roll_t, const double& translation, double& roll_0, const double& theta3_0, const double& theta4_0)
{
  double sign;
  if (INIT_COXA_ANGLE[leg_index] > 0)
  {
    sign = 1.0;
  }
  else
  {
    sign = -1.0;
  }
  
  double b = (1+sign)/2 * M_PI/2;
  
  
  double A = - sign * translation * sin(b - sign*roll_t) 
  - ( sign*BODY_RADIUS*sin(INIT_COXA_ANGLE[leg_index]) + COXA_LENGTH )
  * ( sin(b-sign*roll_t) 
  - sin(b-sign*roll_0) )
  + FEMUR_LENGTH*sin(theta3_0+theta4_0) + TIBIA_LENGTH*cos(theta4_0);
  
  double B = - sign * translation * cos(b - sign*roll_t) 
  - ( sign*BODY_RADIUS*sin(INIT_COXA_ANGLE[leg_index]) + COXA_LENGTH )
  * ( cos(b-sign*roll_t) 
  - cos(b-sign*roll_t) )
  + FEMUR_LENGTH*cos(theta3_0+theta4_0) - TIBIA_LENGTH*sin(theta4_0);
  
  double C = FEMUR_LENGTH;
  
  double D = TIBIA_LENGTH;
  
  double x = 2 * atan( 
    ( 2 * A * C - sqrt( 4 * pow(C, 2) * pow(D, 2) - pow((A*A + B*B - C*C - D*D), 2) ) )
    /
    ( A*A + pow((B+C), 2) - D*D )
  );
  
  double y = 2 * atan( 
    ( -2 * B * D + sqrt( 4 * pow(C, 2) * pow(D, 2) - pow((A*A + B*B - C*C - D*D), 2) ) )
    /
    (  pow((A+D), 2) + B*B - C*C )
  );
  
  *joint = sign * M_PI/2 - INIT_COXA_ANGLE[leg_index];
  *(joint+2) = x - y;
  *(joint+3) = y;
  *(joint+1) = b - sign * roll_t - *(joint+2) - *(joint+3);
  
}















