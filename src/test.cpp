#include "climb_up.h"
// #include <std_msgs/Float64.h>

/*int main(int argc, char ** argv)
{
  ros::init(argc, argv, "clim_up_test");
  Climb_up climb_up;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  
  vector< vector<Climb_up::inter_position> > position(4);
  Climb_up::inter_position pos;
  int leg_index;
  ROS_INFO("leg_index(0~5): ");
  std::cin>>leg_index;
  double distance, feet_height;
  double theta[4];
  
  ROS_INFO("distance: ");
  std::cin>>distance;
  ROS_INFO("theta2: ");
  std::cin>>theta[1];
  theta[1] = M_PI*theta[1]/180.0;
  while(!climb_up.climb_up_calculate(&leg_index, &distance, &theta[1], &theta[0], &theta[2], &theta[3], &feet_height))
  {
    return 0;
  }
  ROS_INFO("theta1: %f", theta[0]);
  ROS_INFO("theta2: %f", theta[1]);
  ROS_INFO("theta3: %f", theta[2]);
  ROS_INFO("theta4: %f", theta[3]);
  ROS_INFO("feet_height: %f", feet_height);
  
  
  for (int i = 0; i < 4; i++)
  {
    pos.position = 0;
    pos.speed = 0;
    position[i].push_back(pos);
    
    pos.position = theta[i]/2;
    pos.speed = 0.1;
    position[i].push_back(pos);
    
    pos.position = theta[i];
    pos.speed = 0;
    position[i].push_back(pos);
  }
  double tf = 2.0;
  climb_up.interpolation_equation(4, position, tf);
  climb_up.publish_interprolation_joint_states(leg_index, climb_up.equation_coe, tf);
  climb_up.clear_equation_coe();
}*/

/*int main(int argc, char ** argv)
{
  ros::init(argc, argv, "clim_up_test");
  Climb_up climb_up;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  
  vector< vector<Climb_up::inter_position> > position(4);
  Climb_up::inter_position pos;

  double left_theta[3][4], right_theta[3][4];
  for(int i = 0; i < 3; i++)
  {
    ROS_INFO("input left_theta2: ");
    std::cin>>left_theta[i][1];
  }
  for(int i = 0; i < 3; i++)
  {
    ROS_INFO("input right_theta2: ");
    std::cin>>right_theta[i][1];
  }
  
  
  climb_up.roll_translation_left(0, &left_theta[0][1], 0.0, 0.0, 0.4, 0.5, &left_theta[0][0], &left_theta[0][2], &left_theta[0][3], 0.0);
  climb_up.roll_translation_left(1, &left_theta[1][1], 0.0, 0.0, 0.4,0.5, &left_theta[1][0], &left_theta[1][2], &left_theta[1][3], 0.0);
  climb_up.roll_translation_left(2,& left_theta[2][1], 0.0, 0.0, 0.4,0.5, &left_theta[2][0], &left_theta[2][2], &left_theta[2][3], 0.0);
  
  climb_up.roll_translation_right(3, &right_theta[0][1], 0.0, 0.0, 0.4,0.5, &right_theta[0][0], &right_theta[0][2], &right_theta[0][3], 0.0);
  climb_up.roll_translation_right(4, &right_theta[1][1], 0.0, 0.0, 0.4,0.5, &right_theta[1][0], &right_theta[1][2], &right_theta[1][3], 0.0);
  climb_up.roll_translation_right(5, &right_theta[2][1], 0.0, 0.0, 0.4,0.5, &right_theta[2][0], &right_theta[2][2], &right_theta[2][3], 0.0);
  
  
  for (int i = 0; i < 4; i++)
  {
    pos.position = 0;
    pos.speed = 0;
    position[i].push_back(pos);
    
    pos.position = left_theta[0][i]/2;
    pos.speed = 0.1;
    position[i].push_back(pos);
    
    pos.position = left_theta[0][i];
    pos.speed = 0;
    position[i].push_back(pos);
  }
  double tf = 2.0;
  climb_up.interpolation_equation(4, position, tf);
  climb_up.publish_interprolation_joint_states(0, climb_up.equation_coe, tf);
  climb_up.clear_equation_coe();
  vector< vector<Climb_up::inter_position> > ().swap(position);
  position.resize(4);
  
  for (int i = 0; i < 4; i++)
  {
    pos.position = 0;
    pos.speed = 0;
    position[i].push_back(pos);
    
    pos.position = left_theta[1][i]/2;
    pos.speed = 0.1;
    position[i].push_back(pos);
    
    pos.position = left_theta[1][i];
    pos.speed = 0;
    position[i].push_back(pos);
  }
  climb_up.interpolation_equation(4, position, tf);
  climb_up.publish_interprolation_joint_states(1, climb_up.equation_coe, tf);
  climb_up.clear_equation_coe();
  vector< vector<Climb_up::inter_position> > ().swap(position);
  position.resize(4);
  
  for (int i = 0; i < 4; i++)
  {
    pos.position = 0;
    pos.speed = 0;
    position[i].push_back(pos);
    
    pos.position = left_theta[2][i]/2;
    pos.speed = 0.1;
    position[i].push_back(pos);
    
    pos.position = left_theta[2][i];
    pos.speed = 0;
    position[i].push_back(pos);
  }
  climb_up.interpolation_equation(4, position, tf);
  climb_up.publish_interprolation_joint_states(2, climb_up.equation_coe, tf);
  climb_up.clear_equation_coe();
  vector< vector<Climb_up::inter_position> > ().swap(position);
  position.resize(4);
  
  
  //right
  for (int i = 0; i < 4; i++)
  {
    pos.position = 0;
    pos.speed = 0;
    position[i].push_back(pos);
    
    pos.position = right_theta[0][i]/2;
    pos.speed = 0.1;
    position[i].push_back(pos);
    
    pos.position = right_theta[0][i];
    pos.speed = 0;
    position[i].push_back(pos);
  }
  climb_up.interpolation_equation(4, position, tf);
  climb_up.publish_interprolation_joint_states(3, climb_up.equation_coe, tf);
  climb_up.clear_equation_coe();
  vector< vector<Climb_up::inter_position> > ().swap(position);
  position.resize(4);
  
  for (int i = 0; i < 4; i++)
  {
    pos.position = 0;
    pos.speed = 0;
    position[i].push_back(pos);
    
    pos.position = right_theta[1][i]/2;
    pos.speed = 0.1;
    position[i].push_back(pos);
    
    pos.position = right_theta[1][i];
    pos.speed = 0;
    position[i].push_back(pos);
  }
  climb_up.interpolation_equation(4, position, tf);
  climb_up.publish_interprolation_joint_states(4, climb_up.equation_coe, tf);
  climb_up.clear_equation_coe();
  vector< vector<Climb_up::inter_position> > ().swap(position);
  position.resize(4);
  
  for (int i = 0; i < 4; i++)
  {
    pos.position = 0;
    pos.speed = 0;
    position[i].push_back(pos);
    
    pos.position = right_theta[2][i]/2;
    pos.speed = 0.1;
    position[i].push_back(pos);
    
    pos.position = right_theta[2][i];
    pos.speed = 0;
    position[i].push_back(pos);
  }
  climb_up.interpolation_equation(4, position, tf);
  climb_up.publish_interprolation_joint_states(5, climb_up.equation_coe, tf);
  climb_up.clear_equation_coe();
  vector< vector<Climb_up::inter_position> > ().swap(position);
  position.resize(4);
}*/


/*int main(int argc, char ** argv)
{
  ros::init(argc, argv, "clim_up_test");
  vector< vector<Climb_up::inter_position> > position(24);
  Climb_up climb_up;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  Climb_up::inter_position pos;
 

  double theta[6][4];
  for(int i = 0; i < 3; i++)
  {
    ROS_INFO("input left_theta2: ");
    std::cin>>theta[i][1];
  }
  for(int i = 3; i < 6; i++)
  {
    ROS_INFO("input right_theta2: ");
    std::cin>>theta[i][1];
  }
  
  climb_up.roll_translation_left(0, &theta[0][1], 0, 0.0, 0.4, 0, &theta[0][0], &theta[0][2], &theta[0][3], 0.0);
  climb_up.roll_translation_left(1, &theta[1][1], 0.0, 0.0, 0.4,0, &theta[1][0], &theta[1][2], &theta[1][3], 0.0);
  climb_up.roll_translation_left(2,& theta[2][1], 0.0, 0.0, 0.4,0, &theta[2][0], &theta[2][2], &theta[2][3], 0.0);
  
  climb_up.roll_translation_right(3, &theta[3][1], 0.0, 0.0, 0.4,0, &theta[3][0], &theta[3][2], &theta[3][3], 0.0);
  climb_up.roll_translation_right(4, &theta[4][1], 0.0, 0.0, 0.4,0, &theta[4][0], &theta[4][2], &theta[4][3], 0.0);
  climb_up.roll_translation_right(5, &theta[5][1], 0.0, 0.0, 0.4,0, &theta[5][0], &theta[5][2], &theta[5][3], 0.0);
  
  for(int leg_index = 0; leg_index < 6; leg_index++)
  {
    for(int i = 0; i < 4; i++)
    {
      pos.position = 0;
      pos.speed = 0;
      position[leg_index*4+i].push_back(pos);
      
      pos.position = theta[leg_index][i]/2;
      pos.speed = 0.1;
      position[leg_index*4+i].push_back(pos);
      
      pos.position = theta[leg_index][i];
      pos.speed = 0;
      position[leg_index*4+i].push_back(pos);
    }
  }
    double tf = 2.0;
    climb_up.interpolation_equation(24, position, tf);
    climb_up.publish_interprolation_all_joint_states(climb_up.equation_coe, tf);
    
  
}*/

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "climb_up");
  Climb_up climb_up;
  ros::AsyncSpinner spinner(2);
  spinner.start();
  vector< vector<Climb_up::inter_position> > position(24);
  Climb_up::inter_position pos;
  double joint[6][4];
  
  double roll_t = 0.3;
  double translation = 0;
  double roll_0 = 0;
  
  double theta3_0[6] = {0};
  double theta4_0[6] = {0};
  
/*  theta3_0 = -0.2220;
  theta4_0 = -0.7928;
  climb_up.roll_translation(0, joint[0], roll_t, translation, roll_0, theta3_0, theta4_0);
  
  theta3_0 = -0.8463;
  theta4_0 = -1.4171;
  climb_up.roll_translation(1, joint[1], roll_t, translation, roll_0, theta3_0, theta4_0);
  
  theta3_0 = -0.2220;
  theta4_0 = -0.7928;
  climb_up.roll_translation(2, joint[2], roll_t, translation, roll_0, theta3_0, theta4_0);
  
  theta3_0 = 0;
  theta4_0 = 0;
  climb_up.roll_translation(3, joint[3], roll_t, translation, roll_0, theta3_0, theta4_0);
  
  theta3_0 = 0;
  theta4_0 = 0;
  climb_up.roll_translation(4, joint[4], roll_t, translation, roll_0, theta3_0, theta4_0);
  
  theta3_0 = 0;
  theta4_0 = 0;
  climb_up.roll_translation(5, joint[5], roll_t, translation, roll_0, theta3_0, theta4_0);*/
  


  double distance, feet_height;
  int legs_index;
  
  ROS_INFO("distance: ");
  std::cin>>distance;
  ROS_INFO("theta2: ");
  legs_index = 0;
  std::cin>>joint[legs_index][1];
  joint[legs_index][1] = M_PI*joint[legs_index][1]/180.0;
  while(!climb_up.climb_up_calculate(&legs_index, &distance, &joint[legs_index][1], &joint[legs_index][0], &theta3_0[legs_index], &theta4_0[legs_index], &feet_height))
  {
    return 0;
  }
  
  ROS_INFO("distance: ");
  std::cin>>distance;
  ROS_INFO("theta2: ");
  legs_index = 1;
  std::cin>>joint[legs_index][1];
  joint[legs_index][1] = M_PI*joint[legs_index][1]/180.0;
  while(!climb_up.climb_up_calculate(&legs_index, &distance, &joint[legs_index][1], &joint[legs_index][0], &theta3_0[legs_index], &theta4_0[legs_index], &feet_height))
  {
    return 0;
  }
  
  ROS_INFO("distance: ");
  std::cin>>distance;
  ROS_INFO("theta2: ");
  legs_index = 2;
  std::cin>>joint[legs_index][1];
  joint[legs_index][1] = M_PI*joint[legs_index][1]/180.0;
  while(!climb_up.climb_up_calculate(&legs_index, &distance, &joint[legs_index][1], &joint[legs_index][0], &theta3_0[legs_index], &theta4_0[legs_index], &feet_height))
  {
    return 0;
  }
  

  for(int leg_index=0; leg_index<6; leg_index++)
  {
    climb_up.roll_translation(leg_index, joint[leg_index], roll_t, translation, roll_0, theta3_0[leg_index], theta4_0[leg_index]);
  }
  
  for(int i=0; i<6; i++)
  {
    for(int j=0; j<4; j++)
    {
      std::cout<<joint[i][j]<<" ";
    }
    std::cout<<std::endl;
  }
  
  
  for(int leg_index = 0; leg_index < 6; leg_index++)
  {
    for(int i = 0; i < 4; i++)
    {
      pos.position = 0;
      pos.speed = 0;
      position[leg_index*4+i].push_back(pos);
      
//       pos.position = joint[leg_index][i]/2;
//       pos.speed = 0;
//       position[leg_index*4+i].push_back(pos);
      
      pos.position = joint[leg_index][i];
      pos.speed = 0;
      position[leg_index*4+i].push_back(pos);
    }
  }
  
  double tf = 2.0;
  climb_up.interpolation_equation(24, position, tf);
  climb_up.publish_interprolation_all_joint_states(climb_up.equation_coe, tf);
  
  
}


