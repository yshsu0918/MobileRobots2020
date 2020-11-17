#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include "wiringPi.h"

using namespace std;
int pin=7;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "gpio_test_node");
	ros::NodeHandle node_obj;
	ros::Rate loop_rate(10);
    //setenv("WIRINGPI_GPIOMEM","1",1);
	wiringPiSetup();
	pinMode(pin,INPUT);
    int light=0;
	while (ros::ok())
	{
        light=digitalRead(pin);
        ROS_INFO("light__: %d",light);
		loop_rate.sleep();
	}
	return 0;
}
