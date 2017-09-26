#include <iostream>
#include <vector>
#include <time.h>
#include <math.h>


#include "ros/ros.h"
#include <std_msgs/Header.h>
#include <sensor_msgs/FluidPressure.h>
#include <dwyer_sensor_driver/releaseVel.h>
#include <weather_board_driver/wb_data.h>


double p;
double T=0;
double rh;

void pressureCallback(const sensor_msgs::FluidPressure &msg){
  p = msg.fluid_pressure;
}

void wbCallback(const weather_board_driver::wb_data msg){
  T = msg.temperature+273.15;  // Kelvin
  rh = msg.humidity;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "release_vel_node");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<dwyer_sensor_driver::releaseVel>("release_vel", 1);
  ros::Subscriber subDwyer = n.subscribe("dwyer_pressure", 1, pressureCallback);
  ros::Subscriber subWB = n.subscribe("wb_data", 1, wbCallback);
  
  int lr;
  n.getParam("/loopR", lr);
  double K_l;      // minor losses coefficent
  n.getParam("/Kl", K_l);
  double r;
  n.getParam("/radi", r);
  r = r*0.0254; //3.*0.0254; // radi of outlet
  double A   = M_PI*pow(r,2.);// area of outlet
  double rho = 1.225;    // kg/m^3  
  double R_specific = 287.058; //461.495; // J/kg*K

  //std::cout<<A<<std::endl;
  ROS_INFO("[relvel] radius of outlet: %f (m)", r);
  ROS_INFO("[relvel] Area of outlet: %f (m^2)", A);
  
  ros::Rate loop_rate(lr);
  
  while (ros::ok()) {

    if (p>0){
      if (T==0){T=294.15;}       
      rho = (p+101325)/(R_specific*T); // kg/m^3
      //ROS_INFO("[relvel] p: %f", (p+101325));
      //ROS_INFO("[relvel] rho: %f", rho);
      dwyer_sensor_driver::releaseVel msg;    
      msg.header.stamp =ros::Time::now();
      msg.header.frame_id ="source";
      msg.windRate = sqrt(2*p/(rho*(1+K_l)));
      msg.releaseRate = msg.windRate*A*rho;
      
      pub.publish(msg);
    }
	
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
