#include <ros/ros.h>  
#include <iostream>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <cstdlib>
#include "std_msgs/Int32.h"
using namespace std;

int forward = 0, right_forward = 0, left_forward = 0, right = 0, left = 0, status = 0;
std_msgs::Int32 msgL;
std_msgs::Int32 msgR;
ros::Publisher Lmotor_publisher;
ros::Publisher Rmotor_publisher;

//Get laser data callback function
int min_lidar(int number, int range, const sensor_msgs::LaserScan& laser) {
    int min_count = 0;
    for (int i = number - range / 2; i < number + range / 2; i++) {
        if (laser.ranges[i]<0.3) {
            min_count++;
        }
    }
    return min_count;
}


#define MAX_DIRECTIONS 12
#define RAD2DEG(x) ((x)*180./M_PI)
int result[MAX_DIRECTIONS];

bool InInterval(float mid, float range, float ask){
	return abs(mid - ask) < range || abs(mid - ask + 360) < range || abs(mid - ask - 360) < range;
}
float min_[MAX_DIRECTIONS];
void get_laser_callback(const sensor_msgs::LaserScan::ConstPtr& scan) {

    int count = scan->scan_time / scan->time_increment;
    // ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    // ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
	
	// int left = 0,left_forward=0, right_forward=0, right =0 , forward = 0;
	
	float a[MAX_DIRECTIONS];
	int b[MAX_DIRECTIONS];
	
	for(int i =0;i<MAX_DIRECTIONS;i++){
		a[i] = 0;
		b[i] = 0;
		min_[i] = 1000;
	}
	
	for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
		if(degree < 0){
			degree += 360;
		}
		//cout << degree << endl;
		float empty_pnt = isinf ( scan->ranges[i]) ? 8 : scan->ranges[i];
		int t = int(degree/ (360/MAX_DIRECTIONS));
		
		a[ t ] += empty_pnt;
		
		//b[ int(degree/ (360/MAX_DIRECTIONS)) ] ++;
		if(min_[t] > empty_pnt){
			min_[t] = empty_pnt;
		}
    }

	float buf[MAX_DIRECTIONS];
	//system("clear");
	for(int i =0;i< MAX_DIRECTIONS;i++){
		// cout << "a " << i << ":"<< a[i] << endl; 
		// cout << "b " << i << ":"<< b[i] << endl; 
		//cout << "a/b " << i << ":"<< a[i]/float(b[i]) << endl; 
		// cout << i*30 << "-" << i*30+30 << ":"<< min_[i] << endl;
	}
	
	// for(int i= 0;i<MAX_DIRECTIONS;i++){
		// cout << "dir " << i << ":"<<buf[i] << endl; 
		// result[i] = buf[i] > 3 ? 1: 0;
	// }
	
    // if (left > 3)
        // reuslt = 0
    // if (left_forward > 3)
        // status += 2;
    // if (forward > 3)
        // status += 4;
    // if (right_forward > 3)
        // status += 8;
    // if (right > 3)
        // status += 16;
	
	

    // cout << "ROS Lidar Data" << endl;
    // cout << "Left:" << left << endl;
    // cout << "left_forward:" << left_forward << endl;
    // cout << "Straight ahead:" << forward << endl;
    // cout << "right_forward:" << right_forward << endl;
    // cout << "right:" << right << endl;
    // cout << "status: " << status << endl;
    // cout << "-----------------" << endl;
}


int main(int argc, char** argv)
{
    // Initialize ROS node
    ros::init(argc, argv, "laser");
    ros::NodeHandle n;
    Lmotor_publisher = n.advertise<std_msgs::Int32>("/receive_msgL", 1);
    Rmotor_publisher = n.advertise<std_msgs::Int32>("/receive_msgR", 1);
    ros::Rate loop_rate(1);
    // Subscribe to the ros topic in gazebo and set the callback function
    ros::Subscriber laser_sub = n.subscribe("/scan", 10 , get_laser_callback);
	
	float avoid_distance;
	if(argc > 1)
		avoid_distance = atof(argv[1]);
	else
		avoid_distance = 0.5;
	
    while (ros::ok())
    {
		cout << "--------------------------------- " << endl;
		int final_direction = 0, min_distance = 10000, min_index = -1;
		int current_distance;
		
		int forward_section = MAX_DIRECTIONS/2;
		int start_section = 5;
		int middle_section = 8;
		int end_section = 12;
		for(int i = start_section; i < end_section ; i++){
			if(min_[i] < avoid_distance ){
				cout << "Direction " << i << " is blocked"<< endl;
				continue;
			}
			current_distance = abs( middle_section - i);
			if(min_distance > current_distance){
				min_distance = current_distance;
				min_index = i;
			}
		}
		cout << "The nearest non-blocked direction to central is " << min_index << ".\n";
		if(min_index == -1){
			msgL.data = -50;
			msgR.data = -50;
		}
		else{
			int move_unit = 10;
			int Q = min_index - start_section;
			msgR.data = (forward_section - Q + 1) * move_unit;
			msgL.data = (Q + 1) * move_unit;

		}
		cout << "Go L/R: " << msgL.data << "/" << msgR.data <<endl;
		
		if(argc > 1 && avoid_distance < 0){
		}
		else{
			Rmotor_publisher.publish(msgR);
			Lmotor_publisher.publish(msgL);
		}
        
        
        ros::spinOnce();
        loop_rate.sleep();
    // callback rotation
    }
    return 0;
}
