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

#ifndef _Base_H_
#define _Base_H_


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


class Base {
	public:
		//methods
		Base(); //constructor
		//members

        // config stuff
        std::vector<std::string> name;   
        

        
        double gain_P;    
        double gain_I; 
        double gain_D; 
        
        int joint_encoder_res;
        double joint_encoder_offset;
        std::vector<int> mosfet_config;
          


        int cur_max;
        int cur_min;
          
        double vel_max;
        double vel_min;

        //Motor stuff
  			std::vector<bool> torque;
  			std::vector<int> motor_direction;
  			
  
        std::vector<double> pos;
        std::vector<double> vel;
        std::vector<double> eff;

        std::vector<double> des_pos;
        std::vector<double> des_vel;
        std::vector<double> des_eff;
        
				//FPGA stuff
        std::vector<uint8_t> error_code; 
				std::vector<uint8_t>  udp_states;      
        std::vector<int> port;	
        
        //from Base
        std::vector<int> raw_pos;        
        std::vector<int> raw_pos_revolutions;  
        std::vector<int> raw_encoder;
        std::vector<int> raw_vel;
        std::vector<double> vel_filtered;  
        std::vector<int> pwm;				
        
	        
	
	
	
        //PID control stuff.
        std::vector<int> ffc;        	       
        int get_control;
        
        int LED;
  
  			double omnidrive_limit(double x, double max, double min);
  
        
	
	private:

	
};



#endif // _Base_H_
