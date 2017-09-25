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
#include <std_msgs/Header.h>
#include <sensor_msgs/FluidPressure.h>


int main(int argc, char **argv)
{

  ros::init(argc, argv, "weather_board_driver");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<sensor_msgs::FluidPressure>("dwyer_pressure", 1);

  FILE *fd = NULL;  
  
  int lr=100;
  ros::Rate loop_rate(lr);
  int adcValue;
  double i=0, p=0;
  
  std::cout<<fd<<std::endl;
  
  while (ros::ok()) {

    // requires opening and closing to work.
    fd = fopen("/sys/class/saradc/ch0", "rb");
    char buf[10] = {0};
    fread(buf,sizeof(char),10,fd);    
    fclose(fd);
    
    i = atof(buf)*0.02147;
    p = (i*.03125-.125)*248.84;
    //std::cout<<"adc: "<<atof(buf)<<" units"<<"   i: "<<i<<" mA"<<"   p: "<<p<<" Pa"<<std::endl;

    sensor_msgs::FluidPressure msg;
    msg.header.stamp =ros::Time::now();
    msg.header.frame_id ="source";    
    msg.fluid_pressure = p;

    pub.publish(msg);
    
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
