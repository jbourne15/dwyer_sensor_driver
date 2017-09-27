//#include <wiringPiI2C.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
#include <cstdio>
#include <vector>
#include <numeric>
#include <time.h>
#include <wiringPi.h>

#include "ros/ros.h"
#include "lowPass.h"
#include <std_msgs/Header.h>
#include <sensor_msgs/FluidPressure.h>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "dwyer_sensor");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<sensor_msgs::FluidPressure>("dwyer_pressure", 1);
  ros::Publisher filPub = n.advertise<sensor_msgs::FluidPressure>("dwyer_pressure_filtered", 1);

  FILE *fd = NULL;  
  
  int lr;
  n.getParam("/loopR", lr);

  double tc; // time constant;
  n.getParam("/tc", tc);
  
  
  ros::Rate loop_rate(lr);
  int adcValue;
  double i=0, p=0;


  int biasCheck;
  n.getParam("/biasC", biasCheck);
  double bias = 0;
  
  LOWPASS lp(tc);
  
  //std::cout<<fd<<std::endl;
  int c=0;
  while (ros::ok()) {

    // requires opening and closing to work.
    fd = fopen("/sys/class/saradc/ch0", "rb");
    char buf[10] = {0};
    fread(buf,sizeof(char),10,fd);    
    fclose(fd);

    if (c>biasCheck){            
      i = atof(buf)*bias; // bias is average value of pressure during beginning of startup
      p = (i*.03125-.125)*248.84; // (Pa)
      //std::cout<<"adc: "<<atof(buf)<<" units"<<"   i: "<<i<<" mA"<<"   p: "<<p<<" Pa"<<std::endl;

      
      sensor_msgs::FluidPressure msg;
      msg.header.stamp =ros::Time::now();
      msg.header.frame_id ="source";    
      msg.fluid_pressure = p;      
      pub.publish(msg);

      sensor_msgs::FluidPressure msgF;
      msgF.header.stamp =ros::Time::now();
      msgF.header.frame_id ="source";    
      msgF.fluid_pressure = lp.filter(msg.fluid_pressure);
      filPub.publish(msgF);
      
    }
    else{      
      bias+=atof(buf);
      //std::cout<<atof(buf)<<std::endl;
      if (c==biasCheck){
	//ROS_INFO("[dwyer] bias: %f", bias);
	bias = bias /(biasCheck+1);
	ROS_INFO("[dwyer] bias: %f", bias);
	bias = 4.0/bias;
	ROS_INFO("[dwyer] bias: %f", bias);
      }
      c++;
    }
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
