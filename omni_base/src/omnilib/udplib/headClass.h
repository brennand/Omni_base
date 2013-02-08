/*
 *  Quadrocopter.h
 *  Quadrocopter
 *
 *  Created by Bren on 6/1/11.
 *  Copyright 2011 TUM. All rights reserved.
 *
 */


/*
 *  Robot.h
 *  UDP
 *
 *  Created by Bren on 2/17/10.
 *  Copyright 2010 TUM. All rights reserved.
 *
 */

#ifndef _Head_H_
#define _Head_H_


//THese defines are used to say what mode the communication is in.  Full controller is used to control the robot in normal mode.


//FPGA Controller Mode
#define off                 0x00
#define full_Controller     0x01
#define update_Values       0x02
#define Test_UDP            0x03

#define LED_switch					0x09

//Communication who?
#define FPGA				0x01

#include <stdint.h>
#include <string>
#include <vector>


class Head {
	public:
		//methods
		Head(); //constructor
		//members

        // config stuff
        std::vector<std::string> name;   
        
        std::vector<double> gain_P;    
        std::vector<double> gain_I; 
        std::vector<double> gain_D; 
        
        std::vector<int> joint_encoder_offset;
        std::vector<int> joint_encoder_res;
        std::vector<int> mosfet_config;
          
        std::vector<int> cur_max;
        std::vector<int> cur_min;
          
        std::vector<double> angle_max;
        std::vector<double> angle_min;

        //Motor stuff
        std::vector<double> pos;
        std::vector<double> vel;
        std::vector<double> eff;

        std::vector<double> des_pos;
        std::vector<double> des_vel;
        std::vector<double> des_eff;

        //from head
        std::vector<int> raw_encoder;
        std::vector<int> raw_vel;      
        std::vector<double> vel_filtered;  
        std::vector<int> pwm;				
	
        //PID control stuff.
        std::vector<int> ffc;        	       
        int get_control;
        
        int LED;
        
	
	private:

	
};



#endif // _Head_H_
