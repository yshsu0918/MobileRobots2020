#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include "wiringPi.h"
#include <ctime>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>

using namespace std;
int LightPin=4;

//TODO

//https://pinout.xyz/pinout/wiringpi

int TouchPinL=7;
int TouchPinR=27;
int TouchPinB=28;

int IRReceiverPin = 29;
double goal_ir_ratio = 0.17;
#define COLLIDE 0 
#define BACK 1
#define FIND_LED 2
#define SPIN 3
#define SPINFIX 6
#define FORWARD 4
#define END 5
#define CAPTURE_LED 7
#define FIND_IR 8


#define ON_LIGHT 0
#define OFF_LIGHT 1

#define WAIT 4
#define GO 5

std_msgs::Int32 msgL;
std_msgs::Int32 msgR;
ros::Publisher Lmotor_publisher ;
ros::Publisher Rmotor_publisher ;


void MoveBackLeft(ros::Publisher& Lmotor_publisher, ros::Publisher& Rmotor_publisher){
	msgL.data = -50;
	msgR.data = -100;
	Lmotor_publisher.publish(msgL);
	Rmotor_publisher.publish(msgR);
}
void MoveBackRight(ros::Publisher& Lmotor_publisher, ros::Publisher& Rmotor_publisher){
	msgL.data = -100;
	msgR.data = -50;
	Lmotor_publisher.publish(msgL);
	Rmotor_publisher.publish(msgR);
}

void MoveStop(ros::Publisher& Lmotor_publisher, ros::Publisher& Rmotor_publisher){
	msgL.data = 0;
	msgR.data = 0;
	Lmotor_publisher.publish(msgL);
	Rmotor_publisher.publish(msgR);
}
void MoveSpin(ros::Publisher& Lmotor_publisher, ros::Publisher& Rmotor_publisher){
	msgL.data = -25;
	msgR.data = 25;
	Lmotor_publisher.publish(msgL);
	Rmotor_publisher.publish(msgR);
}
void MoveSpinRev(ros::Publisher& Lmotor_publisher, ros::Publisher& Rmotor_publisher){
	msgL.data = 15;
	msgR.data = -15;
	Lmotor_publisher.publish(msgL);
	Rmotor_publisher.publish(msgR);
}
void MoveFoward(ros::Publisher& Lmotor_publisher, ros::Publisher& Rmotor_publisher){
	msgL.data = 130;
	msgR.data = 130;
	Lmotor_publisher.publish(msgL);
	Rmotor_publisher.publish(msgR);
}
void MoveBackward(ros::Publisher& Lmotor_publisher, ros::Publisher& Rmotor_publisher){
	msgL.data = -80;
	msgR.data = -80;
	Lmotor_publisher.publish(msgL);
	Rmotor_publisher.publish(msgR);
}


// void signalHandler( int signum ) {
	// cout << "Interrupt signal (" << signum << ") received.\n";
	// MoveStop(Lmotor_publisher, Rmotor_publisher);
	// // cleanup and close up stuff here  
	// // terminate program  

	// exit(signum);  
// }

float calculate_IR_ratio(ros::Rate& loop_rate){
  double lastms,count_0=0,count_1=0,beacon=0;
	//ros::Rate loop_rate(25);
  while(ros::ok()){
  lastms=ros::Time::now().toSec();
   while (ros::Time::now().toSec()-lastms<0.04){
     int IR_data = digitalRead(IRReceiverPin);
     if(IR_data == 0)
          count_0++;
     else if(IR_data == 1)
          count_1++;
    }
    beacon = (count_0)/(count_1+count_0);
    //ROS_INFO("beacon read : %1f",beacon);
    count_0=0;
    count_1=0;  
    loop_rate.sleep();
  }
  return beacon;
}	


int _light;
void number_callback(const std_msgs::Int32& msg){
	//ROS_INFO("Received [%d]", msg.data);
  _light = msg.data;
}
int _light_val = 700;
bool IsLightOn(){
  return _light < _light_val;
}

int main(int argc, char **argv)
{
	// signal(SIGINT, signalHandler);  
	ros::init(argc, argv, "checkpoint4_node");

	ros::NodeHandle node_obj;

	Lmotor_publisher = node_obj.advertise<std_msgs::Int32>("/receive_msgL", 1);
	Rmotor_publisher = node_obj.advertise<std_msgs::Int32>("/receive_msgR", 1);
	
  ros::Rate loop_rate(10);
	
	int number_count = 0,velL=0,velR=0;
    //setenv("WIRINGPI_GPIOMEM","1",1);
	wiringPiSetup();
	
	pinMode(LightPin,INPUT);
	pinMode(TouchPinL,INPUT);
	pinMode(TouchPinR,INPUT);
	pinMode(TouchPinB,INPUT);
	pinMode(IRReceiverPin,INPUT);

	
	string STATUS_STRING[] = {"COLLIDE", "BACK", "FIND_LED", "SPIN", "FORWARD","END", "FIND_IR","CAPTURE_LED"};
	//Current state of each pin
    int _TouchPinL=0, _TouchPinR=0, _TouchPinB=0;
    double _IRReceiverPin=0,L_IRReceiverPin=0;
	//Last state of each pin
    int L_light=0, L_TouchPinL=0, L_TouchPinR=0, L_TouchPinB=0, L_status=0 ;
	
	int status;
	if(argc == 2) //if argc == 2 DEBUG mode is ON.
		status = END;
	else
		status = FORWARD;
	
  if(argc == 3){
    
    _light_val = atoi(argv[1]);
    cout << "set _light_val to " << _light_val;
  }
 
	int last_state_number_count = 0;
	int SPIN_random, randomed=0;
  int backstatus = 0;
	
	MoveStop(Lmotor_publisher, Rmotor_publisher);
	cout << "init status " << STATUS_STRING[status].c_str() << endl;
	ros::Subscriber number_subscribe = node_obj.subscribe("/ard2rpi", 1, number_callback);
  string current_target("LED");
  
	while (ros::ok())
	{
		L_light = _light;
		L_TouchPinL = _TouchPinL;
		L_TouchPinR = _TouchPinR;
		L_TouchPinB = _TouchPinB;
		L_IRReceiverPin = _IRReceiverPin;
		L_status = status;
		
		//_light=digitalRead(LightPin);
		_TouchPinL=digitalRead(TouchPinL);
		_TouchPinR=digitalRead(TouchPinR);
		_TouchPinB=digitalRead(TouchPinB);
		//_IRReceiverPin=digitalRead(IRReceiverPin);
		
		if(argc == 2){
			ROS_INFO("LightPin: %d",_light);
			ROS_INFO("TouchPinL: %d",_TouchPinL);
			ROS_INFO("TouchPinR: %d",_TouchPinR);
			ROS_INFO("TouchPinB: %d",_TouchPinB);
			ROS_INFO("IRReceiverPin: %d",_IRReceiverPin);
		}
    
		if ( L_TouchPinL != _TouchPinL || 
		L_TouchPinR != _TouchPinR || L_TouchPinB != _TouchPinB || 
		status != L_status ){		
			ROS_INFO("LightPin: %d",_light);
			ROS_INFO("TouchPinL: %d",_TouchPinL);
			ROS_INFO("TouchPinR: %d",_TouchPinR);
			ROS_INFO("TouchPinB: %d",_TouchPinB);
			ROS_INFO("IRReceiverPin: %d",_IRReceiverPin);
			ROS_INFO("number_count: %d", number_count);
			ROS_INFO("last_state_number_count: %d", last_state_number_count);
			ROS_INFO("status: %d, %s", status, STATUS_STRING[status].c_str());
		}
		if( _TouchPinB && current_target == "LED" ){
			status=CAPTURE_LED;
      current_target = "IR";
		}
		switch(status){
			case COLLIDE:
				if(number_count - last_state_number_count > 3){
					MoveStop(Lmotor_publisher, Rmotor_publisher);
					status = current_target == "LED" ? FIND_LED : FIND_IR ;
					last_state_number_count = number_count;
				}
				else{
					cout << "In COLLIDE" << endl;
					MoveStop(Lmotor_publisher, Rmotor_publisher);
          string Q("BACKWARD");
          if(_TouchPinL && _TouchPinR){
            backstatus = 0;
          }                                     
          else if(_TouchPinL){
            backstatus = 2;
            Q = "BACKRIGHT";
          }
          else if(_TouchPinR){
            backstatus = 1;
            Q = "BACKLEFT";
          }
          else{
            backstatus = 0;
          }

          cout << "In COLLIDE, backstatus= " << backstatus << "," << Q << endl;
					status = BACK;
					last_state_number_count = number_count;
				}
				break;
			case BACK:
				if(number_count - last_state_number_count > 13){
					MoveStop(Lmotor_publisher, Rmotor_publisher);
					status = current_target == "LED" ? FIND_LED : FIND_IR ;
					last_state_number_count = number_count;
				}
				else{
					//cout << "Inside BACK" << endl;
          switch(backstatus){
            case 0:
              //cout << "Inside MoveBackward" << endl;
              MoveBackward(Lmotor_publisher, Rmotor_publisher);
              break;
            case 1: //_L == 0
              //cout << "Inside MoveBackRight" << endl;
              MoveBackRight(Lmotor_publisher, Rmotor_publisher);
              break;
            case 2: //_R == 0
              //cout << "Inside MoveBackLeft" << endl;
              MoveBackLeft(Lmotor_publisher, Rmotor_publisher);
              break;
            default:
              MoveBackward(Lmotor_publisher, Rmotor_publisher);
          }
				}
				break;
			case FIND_LED:

				if( IsLightOn() ){
					cout << "FIND LED!!" << endl;
					MoveStop(Lmotor_publisher, Rmotor_publisher);
					
					last_state_number_count = number_count;
					status = SPINFIX;
					cout << "Turn to SPINFIX" << endl;
				}
				else if(_TouchPinL || _TouchPinR){
					cout << "Collide while spin" << endl;
					MoveStop(Lmotor_publisher, Rmotor_publisher);
					last_state_number_count = number_count;
					status = COLLIDE;
					cout << "Turn to COLLIDE" << endl;
				}
				else if(number_count - last_state_number_count > 50){
					cout << "CANNOT FIND LED" << endl;
					MoveStop(Lmotor_publisher, Rmotor_publisher);
					last_state_number_count = number_count;
					status = SPIN;
					cout << "Turn to SPIN" << endl;
				}
				else{
					MoveSpin(Lmotor_publisher, Rmotor_publisher);
					status = FIND_LED;
				}
				break;
			case SPINFIX:
				if(number_count - last_state_number_count > 5){
					cout << "REV finish" << endl;
					MoveStop(Lmotor_publisher, Rmotor_publisher);
					last_state_number_count = number_count;
					status = FORWARD;
					cout << "Turn to FORWARD" << endl;
				}
				else{
					MoveSpin(Lmotor_publisher, Rmotor_publisher);
				}
				break;
			case SPIN:
				if( !randomed){
					SPIN_random = rand()%15+1;
					randomed = 1;
          cout << "Random Spin " << SPIN_random << endl;
				}

				if(_TouchPinL || _TouchPinR){
					cout << "Collide while Random spin" << endl;
					MoveStop(Lmotor_publisher, Rmotor_publisher);
					last_state_number_count = number_count;
					status = COLLIDE;
					cout << "Turn to COLLIDE" << endl;
				}
				else if(number_count - last_state_number_count > SPIN_random){
					MoveStop(Lmotor_publisher, Rmotor_publisher);
					last_state_number_count = number_count;
					status = FORWARD;
					cout << "Turn to FORWARD" << endl;
					randomed = 0;
				}
				else{
					MoveSpin(Lmotor_publisher, Rmotor_publisher);
				}
				break;
			case FORWARD:
				if(_TouchPinL || _TouchPinR ){
					last_state_number_count = number_count;
					status=COLLIDE;
					cout << "Forward Collide"<< endl;
				}
				else if(number_count - last_state_number_count > 80){
					last_state_number_count = number_count;
					status=current_target == "LED" ? FIND_LED : FIND_IR ;
					cout << "Turn to FINDLED" << endl;
				}
				else{
					MoveFoward(Lmotor_publisher, Rmotor_publisher);
				}
				break;
			case CAPTURE_LED:
				cout << "CAPTURE_LED" << endl;
        status = FIND_IR;
				break;
      case FIND_IR:
        cout << "FIND_IR" << endl;
        _IRReceiverPin=calculate_IR_ratio(loop_rate);
        if( (_IRReceiverPin>goal_ir_ratio-0.03)||(_IRReceiverPin<goal_ir_ratio+0.03)){
					cout << "FIND IR!!" << endl;
					MoveStop(Lmotor_publisher, Rmotor_publisher);
					last_state_number_count = number_count;
					status = SPINFIX;
					cout << "Turn to SPINFIX" << endl;
				}
        else if(_TouchPinL || _TouchPinR){
					cout << "Collide while spin" << endl;
					MoveStop(Lmotor_publisher, Rmotor_publisher);
					last_state_number_count = number_count;
					status = COLLIDE;
					cout << "Turn to COLLIDE" << endl;
				}
				else if(number_count - last_state_number_count > 50){
					cout << "CANNOT FIND ir" << endl;
					MoveStop(Lmotor_publisher, Rmotor_publisher);
					last_state_number_count = number_count;
					status = SPIN;
					cout << "Turn to SPIN" << endl;
				}
				else{
					MoveSpin(Lmotor_publisher, Rmotor_publisher);
					status = FIND_IR;
				} 
        break;
		}
		number_count ++;
   
    
    ros::spinOnce();
        
		//calculate_IR_ratio(loop_rate);
		
		if(number_count > 1000){
			MoveStop(Lmotor_publisher, Rmotor_publisher);
			break;
		}
		loop_rate.sleep();
	}
	return 0;
}



