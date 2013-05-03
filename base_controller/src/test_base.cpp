/*
 * Example
 * Copyright (C) 201
 * 
 */

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

#include <string>
#include <vector>
#include <math.h>

#include <XmlRpcValue.h>

#include "time.h"

using namespace XmlRpc;
using namespace std;

#define PS3_BUTTON_SELECT            0
#define PS3_BUTTON_STICK_LEFT        1
#define PS3_BUTTON_STICK_RIGHT       2
#define PS3_BUTTON_START             3
#define PS3_BUTTON_CROSS_UP          4
#define PS3_BUTTON_CROSS_RIGHT       5
#define PS3_BUTTON_CROSS_DOWN        6
#define PS3_BUTTON_CROSS_LEFT        7
#define PS3_BUTTON_REAR_LEFT_2       8
#define PS3_BUTTON_REAR_RIGHT_2      9
#define PS3_BUTTON_REAR_LEFT_1       10
#define PS3_BUTTON_REAR_RIGHT_1      11
#define PS3_BUTTON_ACTION_TRIANGLE   12
#define PS3_BUTTON_ACTION_CIRCLE     13
#define PS3_BUTTON_ACTION_CROSS      14
#define PS3_BUTTON_ACTION_SQUARE     15
#define PS3_BUTTON_PAIRING           16

#define PS3_AXIS_STICK_LEFT_LEFTWARDS    0
#define PS3_AXIS_STICK_LEFT_UPWARDS      1
#define PS3_AXIS_STICK_RIGHT_LEFTWARDS   2
#define PS3_AXIS_STICK_RIGHT_UPWARDS     3
#define PS3_AXIS_BUTTON_CROSS_UP         4
#define PS3_AXIS_BUTTON_CROSS_RIGHT      5
#define PS3_AXIS_BUTTON_CROSS_DOWN       6
#define PS3_AXIS_BUTTON_CROSS_LEFT       7
#define PS3_AXIS_BUTTON_REAR_LEFT_2      8
#define PS3_AXIS_BUTTON_REAR_RIGHT_2     9
#define PS3_AXIS_BUTTON_REAR_LEFT_1      10
#define PS3_AXIS_BUTTON_REAR_RIGHT_1     11
#define PS3_AXIS_BUTTON_ACTION_TRIANGLE  12
#define PS3_AXIS_BUTTON_ACTION_CIRCLE    13
#define PS3_AXIS_BUTTON_ACTION_CROSS     14
#define PS3_AXIS_BUTTON_ACTION_SQUARE    15
#define PS3_AXIS_ACCELEROMETER_LEFT      16
#define PS3_AXIS_ACCELEROMETER_FORWARD   17
#define PS3_AXIS_ACCELEROMETER_UP        18
#define PS3_AXIS_GYRO_YAW                19

void getParamVector_int(ros::NodeHandle, string,vector<int> *);
void getParamVector_double(ros::NodeHandle, string,vector<double> *);
void getParamVector_string(ros::NodeHandle, string,vector<std::string> *);

std::vector<std::string> name;   
std::vector<double> pos;   
std::vector<double> vel;  
std::vector<double> eff;  


std::vector<double> des_pos;   
std::vector<double> des_vel;  
std::vector<double> des_eff; 

double des_x = 0.0;
double des_y = 0.0;
double des_a = 0.0;

sensor_msgs::Joy::ConstPtr joy_msg;

void desiredCallback(const sensor_msgs::JointState::ConstPtr& msg){

    pos = msg->position;
    vel = msg->velocity;
    eff = msg->effort;
}

void ps3dataCallback(const sensor_msgs::Joy::ConstPtr& msg){

	joy_msg = msg;

        des_x = 10.0*joy_msg->axes[PS3_AXIS_STICK_LEFT_UPWARDS];
        des_y = 10.0*joy_msg->axes[PS3_AXIS_STICK_LEFT_LEFTWARDS];
        des_a = 10.0*joy_msg->axes[PS3_AXIS_STICK_RIGHT_LEFTWARDS];

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "base_test");
  ros::NodeHandle n;
  
  //This is the msg listener and the msg caster.
  ros::Publisher js_pub = n.advertise<sensor_msgs::JointState>("/base_interface/command", 1000);
  ros::Publisher tw_pub = n.advertise<geometry_msgs::Twist>("/mecanum_control/des_twist", 1000);


  ros::Subscriber sub = n.subscribe("/base_interface/state", 1000, desiredCallback); //Des_joint_states_Bioloid
  ros::Subscriber ps3_sub = n.subscribe("/joy", 1000, ps3dataCallback);
  
  bool on_off;


  // Get all the variables from the parameter server.
  n.getParam("/robot_param/base/wheels/active", on_off);
  getParamVector_string(n,"/robot_param/base/wheels/name",&name);

    
  ROS_INFO("Number of joints: %d", int(name.size()));
  ROS_INFO("head controller started");

  //Want to communicate at 1Khz
  ros::Rate loop_rate(1000);
  //Initialice the time.
  ros::Time::init();


	

  int counter = 0;
  int change_counter = 0;
	des_pos.clear();   
	des_vel.clear();  
	des_eff.clear(); 

	for (unsigned int i = 0 ; i < name.size() ; ++i){
		des_pos.push_back(0.0);
		des_vel.push_back(0.0);  
		des_eff.push_back(0.0);
	
	}

  while (ros::ok())
  {
    //Send Joint states
//	double des;
    counter ++;
    sensor_msgs::JointState js;
    js.header.seq = counter;
    js.header.stamp = ros::Time::now();
    js.header.frame_id = "/world";

    js.name = name;
    
    geometry_msgs::Twist tw;
   
    
    	tw.linear.x = des_x;
			tw.linear.y = des_y;
			tw.angular.z = des_a;
    tw_pub.publish(tw);
    
    
    //Send a packet then wait for the packet to be returned.

//des = 2*sin(counter*0.0001);


//	des = joy_msg->axes[PS3_AXIS_STICK_LEFT_UPWARDS];

	//des = 10.0;
		
	des_vel.clear();   

    for (unsigned int i = 0 ; i < name.size() ; ++i){
                      des_vel.push_back(des);
    }

	
  change_counter ++;


//	des_pos.clear();
//    for (unsigned int i = 0 ; i < name.size() ; ++i){
//			des_pos.push_back((cos(counter*0.003)));
//    }

    js.position = des_pos;
    js.velocity = des_vel;
    js.effort = des_eff;

   // js_pub.publish(js);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}

// Next three functions returns the paramenter server value.

void getParamVector_string(ros::NodeHandle n, string Var,vector<std::string> *Vec){
 
  	XmlRpcValue gainList;
	n.getParam(Var, gainList);
	ROS_ASSERT(gainList.getType() == XmlRpcValue::TypeArray);

	for (int index = 0; index < gainList.size(); index++)
	{
		ROS_ASSERT(gainList[index].getType() == XmlRpcValue::TypeString);
		Vec->push_back(static_cast<std::string>(gainList[index]));
	}
    
}

void getParamVector_int(ros::NodeHandle n, string Var,vector<int> *Vec){
 
  	XmlRpcValue gainList;
	n.getParam(Var, gainList);
	ROS_ASSERT(gainList.getType() == XmlRpcValue::TypeArray);

	for (int index = 0; index < gainList.size(); index++)
	{
		ROS_ASSERT(gainList[index].getType() == XmlRpcValue::TypeInt);
		Vec->push_back(static_cast<int>(gainList[index]));
	}
    
}


void getParamVector_double(ros::NodeHandle n, string Var,vector<double> *Vec){
 
  	XmlRpcValue List;
	n.getParam(Var, List);
	ROS_ASSERT(List.getType() == XmlRpcValue::TypeArray);

	for (int index = 0; index < List.size(); index++)
	{
		ROS_ASSERT(List[index].getType() == XmlRpcValue::TypeDouble);
		Vec->push_back(static_cast<double>(List[index]));
	}
    
}


