/*
 * Example
 * Copyright (C) 201
 * 
 */

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include <omni_base_msgs/Motor.h>

#include "Communication.h"
#include "baseClass.h"

#include <string>
#include <vector>
#include <math.h>

#include <XmlRpcValue.h>

#include "time.h"

using namespace XmlRpc;
using namespace std;


void getParamVector_int(ros::NodeHandle, string,vector<int> *);
void getParamVector_double(ros::NodeHandle, string,vector<double> *);
void getParamVector_string(ros::NodeHandle, string,vector<std::string> *);
void getParamVector_bool(ros::NodeHandle, string,vector<bool> *);

int watchdogtimer = 0;

char* const IP_FPGA_front  = "10.0.200.168";
char* const IP_FPGA_back   = "11.0.200.167";


Base base;

void desiredCallback(const sensor_msgs::JointState::ConstPtr& msg){
    base.des_pos = msg->position;
    base.des_vel = msg->velocity;
    base.des_eff = msg->effort;
 
 		watchdogtimer = 100; // reset the watchdog as we have a new message.
 
 }





int main(int argc, char **argv)
{
  ros::init(argc, argv, "base_interface");
  ros::NodeHandle n("~");
  
  //This is the msg listener and the msg caster.
  ros::Publisher js_pub = n.advertise<sensor_msgs::JointState>("state", 1000);
  ros::Publisher motor_pub = n.advertise<omni_base_msgs::Motor>("motor_state", 1000);
  ros::Subscriber sub = n.subscribe("command", 1000, desiredCallback);

  bool on_off;
	
  /******************************************************************
  *
  * 			Get all the variables from the parameter server.
  *
  ******************************************************************/

	


  getParamVector_string(n,"/robot_param/base/wheels/name",&base.name);

  n.getParam("/robot_param/base/wheels/joint_encoder_offset", base.joint_encoder_offset);    
  n.getParam("/robot_param/base/wheels/active", on_off);
  n.getParam("/robot_param/base/wheels/LED", base.LED);
  getParamVector_bool(n,"/robot_param/base/wheels/torque", &base.torque);

  n.getParam("/robot_param/base/wheels/gain_P",base.gain_P);
  n.getParam("/robot_param/base/wheels/gain_I",base.gain_I);
  n.getParam("/robot_param/base/wheels/gain_D",base.gain_D);    
  
  n.getParam("/robot_param/base/wheels/joint_encoder_res",base.joint_encoder_res);
  
  n.getParam("/robot_param/base/wheels/cur_max",base.cur_max);
  n.getParam("/robot_param/base/wheels/cur_min",base.cur_min);
  n.getParam("/robot_param/base/wheels/vel_max",base.vel_max);     
  n.getParam("/robot_param/base/wheels/vel_min",base.vel_min); 

  n.getParam("/robot_param/base/wheels/wheel_Circumfrerence",base.Circumfrerence); 


  getParamVector_int(n,"/robot_param/base/wheels/motor_direction",&base.motor_direction);
  getParamVector_int(n,"/robot_param/base/wheels/mosfet_config",&base.mosfet_config);

  getParamVector_int(n,"/robot_param/base/wheels/port",&base.port);

    
    
    
  ROS_INFO("Number of wheels: %d", int(base.name.size()));
  ROS_INFO("base interface started.");
 
  /******************************************************************
  *
  * 			Setup ROS, plus turn the FPGA controller to full control
  *
  ******************************************************************/

  UDP udptoFPGA_front(IP_FPGA_front,base.port[0],base.port[0],0); 
  UDP udptoFPGA_back(IP_FPGA_back,base.port[1],base.port[1],2);
 


	ROS_INFO("Seeting up ROS node.");

  //Set the controller for the base to full control.    
  base.get_control = full_Controller;
  //Want to communicate at 1Khz
  ros::Rate loop_rate(1000);
  //Initialice the time.
  ros::Time::init();

  int counter = 0;

	//Clear all the vectors

	base.ffc.clear();
	base.des_pos.clear();
	base.des_vel.clear();
	base.des_eff.clear();
		
	base.pos.clear();
	base.vel.clear();
	base.eff.clear();

	//Maybe move this into a init function.
	
	base.raw_pos.clear();
	base.raw_pos_revolutions.clear();
	base.raw_encoder.clear();
	base.raw_vel.clear();
	base.pwm.clear();
	base.vel_filtered.clear();
	base.error_code.clear();

	// Setup all the gains and offsets, and setup system
	for (unsigned int i = 0 ; i < base.name.size() ; ++i){
		base.ffc.push_back(0);
		
		base.des_pos.push_back(0.0);
		base.des_vel.push_back(0.0);
		base.des_eff.push_back(0.0);
		
		base.pos.push_back(0.0);
		base.vel.push_back(0.0);
		base.eff.push_back(0.0);
		
		base.raw_pos.push_back(0.0);
		base.raw_pos_revolutions.push_back(0.0);
		base.raw_encoder.push_back(0.0);
		base.raw_vel.push_back(0.0);;
		base.pwm.push_back(0.0);
		base.vel_filtered.push_back(0.0);
		
		base.error_code.push_back(0);		
	}

	base.udp_states.resize(2);	

  /******************************************************************
  *
  * 				Main control loop.
  *
  ******************************************************************/
	ROS_INFO("entering main loop.");
  
  while (ros::ok())
  {
    //Send Joint states

		// decrease the watchdog count
    if(watchdogtimer > 1){
    	watchdogtimer --;
    }
   
    counter ++;
    sensor_msgs::JointState js;
    js.header.seq = counter;
    js.header.stamp = ros::Time::now();
    js.header.frame_id = "/world";

    omni_base_msgs::Motor motor;
    motor.header.seq = counter;
    motor.header.stamp = ros::Time::now();
    motor.header.frame_id = "/world";

    //communiacte with the base
    udptoFPGA_front.Zip_packet(&base);
    if(udptoFPGA_front.Send()){
        if(udptoFPGA_front.Received()){
						base.udp_states[0] = 1;
            udptoFPGA_front.Unzip_packet(&base);
        }else{
					base.udp_states[0] = 0;
				}//udp.Received
    }//udptoFPGA.Send()
    
    //communiacte with the base
     udptoFPGA_back.Zip_packet(&base);
    if( udptoFPGA_back.Send()){
        if( udptoFPGA_back.Received()){

						base.udp_states[1] = 1;
            udptoFPGA_back.Unzip_packet(&base);
        }else{
					base.udp_states[1] = 0;
				}//udp.Received
    }//udptoFPGA.Send()




    base.pos.clear();
    base.vel.clear();
    base.eff.clear();    
        // Convert from raw data to real si unit. 
    for (unsigned int i = 0 ; i < base.name.size() ; ++i){
    	

    		int pos_off_setted = (int)(base.raw_pos[i]-500000);
    
    		double pos_temp1 = base.Circumfrerence * base.raw_pos_revolutions[i];
    		double pos_temp2 = base.Circumfrerence * (double)pos_off_setted/(40*base.joint_encoder_res);
    		
    		
    		//	ROS_INFO("0 = %f , 1 = %f, 2 = %f ,3 = %d /n",base.Circumfrerence, pos_temp1,pos_temp2, pos_off_setted);
    		
        base.pos.push_back((pos_temp1+pos_temp2)*-1*base.motor_direction.at(i));//(base.joint_encoder_offset*1000));
    
    
    
        base.vel.push_back((double)base.raw_vel[i]/400.0*6.28319*base.motor_direction.at(i));  // 400 = ((tick*1000)/10000)/40 - ((tick/s*1khz)/tick/rev)/gear_ratio
        base.eff.push_back(0.0);  
    }

    // Simple PID loop
    base.ffc.clear();    
    for (unsigned int i = 0 ; i < base.name.size() ; ++i){
    	//Make sure the des isn't out of range.

    	base.des_vel.at(i) = base.omnidrive_limit(base.des_vel.at(i), base.vel_max, base.vel_min);
    	

    	//Now do the PID loop.
			double P_error = 0.0;
			int P = 0;
			P_error = base.vel.at(i) - base.des_vel.at(i);

			P = int(P_error * base.gain_P);
			/*
			if(P < 30 and P > -30) {
				P = 0;	
			}
			
			if(P > 1) {
				P += 0;	
			} else if (P < -1) {
				P -= 0;
			}
			*/

			// Take the gain and give it to the motor and also switch the direction of the PWM depending on the motor direction
			base.ffc.push_back(P*base.motor_direction.at(i));

			if(base.torque.at(i) == false || watchdogtimer < 2){
				base.ffc.at(i) = 0;
			}else{
				base.ffc.at(i) = (int)base.omnidrive_limit((double)base.ffc.at(i), (double)base.cur_max, (double)base.cur_min);
			}
			
			



		//ROS_INFO("PWM: %i = %i = %4.2f - %4.2f", i, base.pwm[i] , base.vel[i], base.des_vel[i]);
			
    }



        //Use this for getting the joint offset angle
        if(false) {
        	static int skip = 0;
        	if(skip > 500){
        		skip = 0;
            for (unsigned int i = 0 ; i < base.name.size() ; ++i){
                ROS_INFO("PWM: %i = %i /  %i - Vel= %f , des= %f", i, base.pwm[i], base.ffc[i],base.vel.at(i), base.des_vel.at(i));
            }
        	}else{
        		skip ++;	
        	}
        }
 		
    js.name = base.name;  
    js.position = base.pos;
    js.velocity = base.vel;
    js.effort   = base.eff;

    js_pub.publish(js);

		
		motor.name = base.name;
		motor.ffc = base.ffc;
		motor.pwm = base.pwm;
		motor.vel_raw = base.raw_vel;
    motor.pos_raw = base.raw_pos;		
		motor.revolution_raw = base.raw_pos_revolutions;
		motor.udp_states = base.udp_states;
		motor.error_code = base.error_code;
		
		motor_pub.publish(motor);

    ros::spinOnce();
    loop_rate.sleep();
  }
  
  
	/******************************************************************
	*
  * 				Before exit turn everything off.
  *
  ******************************************************************/
  
  
  
    base.ffc.clear();
    for (unsigned int i = 0 ; i < base.name.size() ; ++i){
       base.ffc.push_back(0);
    }

    //communiacte with the base
     udptoFPGA_back.Zip_packet(&base);
    if( udptoFPGA_back.Send()){
        if( udptoFPGA_back.Received()){
						base.udp_states[0] = 1;
            udptoFPGA_back.Unzip_packet(&base);
        }else{
					base.udp_states[0] = 0;
				}//udp.Received
    }//udptoFPGA.Send()

    //communiacte with the base
    udptoFPGA_front.Zip_packet(&base);
    if(udptoFPGA_front.Send()){
        if(udptoFPGA_front.Received()){
						base.udp_states[1] = 1;
            udptoFPGA_front.Unzip_packet(&base);
        }else{
					base.udp_states[1] = 0;
				}//udp.Received
    }//udptoFPGA.Send()

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

void getParamVector_bool(ros::NodeHandle n, string Var,vector<bool> *Vec){
 
  	XmlRpcValue List;
	n.getParam(Var, List);
	ROS_ASSERT(List.getType() == XmlRpcValue::TypeArray);

	for (int index = 0; index < List.size(); index++)
	{
		ROS_ASSERT(List[index].getType() == XmlRpcValue::TypeBoolean);
		Vec->push_back(static_cast<bool>(List[index]));
	}
    
}
