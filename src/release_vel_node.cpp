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
double tT=0;
double bT=0;
double T=0;
double trh=0;
double brh=0;
double rh=0;

void pressureCallback(const sensor_msgs::FluidPressure &msg){
  p = msg.fluid_pressure;
}

void topwbCallback(const weather_board_driver::wb_data msg){
  tT  = msg.temperature+273.15;  // Kelvin
  trh = msg.humidity;
}

void botwbCallback(const weather_board_driver::wb_data msg){
  bT  = msg.temperature+273.15;  // Kelvin
  brh = msg.humidity;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "release_vel_node");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<dwyer_sensor_driver::releaseVel>("release_vel", 1);
  ros::Subscriber subDwyer = n.subscribe("dwyer_pressure_filtered", 1, pressureCallback);
  ros::Subscriber topsubWB = n.subscribe("top/wb_sensor/wb_data", 1, topwbCallback);
  ros::Subscriber botsubWB = n.subscribe("bot/wb_sensor/wb_data", 1, botwbCallback);
  
  int lr;
  n.getParam("/loopR", lr);
  double K_l;      // minor losses coefficent
  n.getParam("/Kl", K_l);
  double r;
  n.getParam("/radi", r);
  r = r*0.0254; //3.*0.0254; // radi of outlet
  double A   = M_PI*pow(r,2.);// area of outlet
  double R_specific = 287.3; // J/kg*K

  // virtual temperature vars //
  double rho = 1.225; // kg/m^3  
  double q   = 0;     // specific humidity
  double Tv  = 0;     // virtual temperature
  double e   = 0;     // vapor pressure
  double es  = 0;     // saturated vapor pressure
  double es0 = 611;   // (Pa) reference saturation vapor pressure
  double T0  = 273.15;// reference temperature
  double lv  = (2.5*pow(10.0,6.0)); // latent heat of vaporization of water
  double Rv  = 461.5; // gas constant for water vapor
  
  ROS_INFO("[relvel] radius of outlet: %f (m)", r);
  ROS_INFO("[relvel] Area of outlet: %f (m^2)", A);  
  ros::Rate loop_rate(lr);
  
  while (ros::ok()) {

    if (p<0){
      p = 0;
      ROS_INFO_ONCE("Pressure is negative, check power source for dwyer");
    }
    
    //if (tT==0){T=294.15;}
    T   = (tT+bT)/2.0;
    rh  = (trh + brh)/2.0;
    //ROS_INFO("T: %f", T);
    //ROS_INFO("rh: %f", rh);
    
    es  = es0*exp(lv/Rv*(1/T0-1/T));
    e   = rh/100.0*es;
    q   = 0.622*(e/(p+101325));
    Tv  = T*(1.0+0.61*q);
    rho = (p+101325)/(R_specific*Tv); // kg/m^3
    //ROS_INFO("lv, Rv, es, e, q, Tv, rho: %f, %f %f, %f, %f, %f, %f", lv, Rv, es,e,q,Tv,rho);
    
    dwyer_sensor_driver::releaseVel msg;    
    msg.header.stamp =ros::Time::now();
    msg.header.frame_id ="source";
    msg.windRate = sqrt(2*p/(rho*(1+K_l)));
    msg.releaseRate = msg.windRate*A*rho;
    
    pub.publish(msg);
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
