/*
 * This file is part of the libomnidrive project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>,
 *                    Ingo Kresse <kresse@in.tum.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301 USA
 */

/* DESIGN PHASE 1: 
 * - The program is REQUIRED to call omnidrive_odometry often to ensure
 *   reasonable precision.
 */


#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>


#include <math.h>

#include "omnilib.h"



/*****************************************************************************/
/* 												global variables                       						 */
/*****************************************************************************/

// Calculated for the APM-SC05-ADK9 motors with 8" HD AndyMark wheels
const double odometry_constant=626594.7934;  // in ticks/m
const double drive_constant=626594.7934;
double odometry_correction=1.0;
const int max_tick_speed = 666666; // ticks/s : 4000 rpm/ 60s * 10000 ticks/rev



int odometry_initialized = 0;
uint32_t last_odometry_position[4]={0, 0, 0, 0};
double odometry[3] = {0, 0, 0}; //In Tick/s

int status[4];
commstatus_t commstatus;

// This init the controller.
int omnidrive_init(void)
{
	return 0;
}



/*! Jacobians for a mecanum wheels based omnidirectional platform.
 *  The functions jac_forward and jac_inverse convert cartesian
 *  velocities into wheel velocities and vice versa.
 *  For our motors, the order and signs are changed. The matrix C accounts
 *  for this.
 */


// in = vx, vy, rz  (velocities in space of the base)
// out = v0, v1, v2, v3 (wheel velocities)
void jac_forward(double *in, double *out)
{
  // computing:
  //   out = (C*J_fwd) * in
  // with:
  //   J_fwd = [1 -1 -alpha;
  //            1  1  alpha;
  //            1  1 -alpha;
  //            1 -1  alpha]
  //   C     = [0  0  0  1;
  //            0  0 -1  0;
  //            0  1  0  0;
  //           -1  0  0  0 ]

  int i,j;
#define alpha (0.34475 + 0.303475)
  double C_J_fwd[4][3] = {{ 1, -1, alpha},
                          {-1, -1, alpha},
                          { 1,  1, alpha},
                          {-1,  1, alpha}};

  // assert(in  != 0);
  // assert(out != 0);

  for(i=0; i < 4; i++) {
    out[i] = 0;
    for(j=0; j < 3; j++) {
      out[i] += C_J_fwd[i][j]*in[j];
    }
  }
}


// Used to calculate the odometry
// out = vx, vy, rz  (velocities in space of the base)
// in = v0, v1, v2, v3 (wheel velocities)

void jac_inverse(double *in, double *out)
{
	
  // computing:
  //   out = (J_inv*C^-1) * in
  // with:
  //   J_fwd = 1/4*[1 -1 -alpha;
  //                1  1  alpha;
  //                1  1 -alpha;
  //                1 -1  alpha]
  //   C = [0  0  0  1;
  //        0  0 -1  0;
  //        0  1  0  0;
  //       -1  0  0  0 ]

  int i,j;
#define alpha (0.34475 + 0.303475)
  double J_inv_C[3][4] = {{ 0.25,      -0.25,       0.25,      -0.25},
                          {-0.25,      -0.25,       0.25,       0.25},
                          { 0.25/alpha, 0.25/alpha, 0.25/alpha, 0.25/alpha}};

  // assert(in  != 0);
  // assert(out != 0);
	
  for(i=0; i < 3; i++) {
    out[i] = 0;
    for(j=0; j < 4; j++) {
      out[i] += J_inv_C[i][j]*in[j];
    }
  }
}


int omnidrive_drive(double x, double y, double a)
{
 
 /*
  // speed limits for the robot
  double wheel_limit = 1.0 ;//0.8;  // a single wheel may drive this fast (m/s)
  double cart_limit = 0.5 ; //0.5;   // any point on the robot may move this fast (m/s)
  double radius = 0.7;       // (maximum) radius of the robot (m)

  // 0.5 m/s is 1831 ticks. kernel limit is 2000 ticks.

  double corr_wheels, corr_cart, corr;

  //omniwrite_t tar;
  //memset(&tar, 0, sizeof(tar));
 
  double cartesian_speeds[3] = {x, y, a};
  double wheel_speeds[4];
  int i;

  //TODO: check if the robot is up. if not, return immediately
  
  //tar.magic_version = OMNICOM_MAGIC_VERSION;

  // check for limits

  // cartesian limit: add linear and angular parts
  corr_cart = cart_limit / (sqrt(x*x + y*y) + radius*fabs(a));

  // wheel limit: for one wheel, x,y and a always add up
  corr_wheels = wheel_limit / (fabs(x) + fabs(y) + fabs(a));

  // get limiting factor as min(1, corr_cart, corr_wheels)
  corr = (1 < corr_cart) ? 1 : ((corr_cart < corr_wheels) ? corr_cart : corr_wheels);

  jac_forward(cartesian_speeds, wheel_speeds);

  for(i = 0; i < 4; i++) {
    tar.target_velocity[i] = wheel_speeds[i] * corr * drive_constant;
    //tar.torque_set_value[i] = 0.0;
  }

  // Let the kernel know the velocities we want to set. 

	//Pass these back
  //omni_write_data(tar);
*/
  return 0;
}

void omnidrive_set_correction(double drift)
{
  odometry_correction = drift;
}

int omnidrive_odometry(double w1,double w2,double w3,double w4,double *x, double *y, double *a)
{

//  omniread_t cur; // Current velocities / torques / positions 
  int i;
  double d_wheel[4], d[3], ang;

	double position[4];
	position[0] = w1;
	position[1] = w2;	
	position[2] = w3;
	position[3] = w4;

  /* Read data from kernel module. */
 // cur = omni_read_data();
	/*
  // copy status values
  for(i=0; i < 4; i++)
    status[i] = cur.status[i];

  for(i=0; i < 4; i++) {
    commstatus.slave_state[i] = cur.slave_state[i];
    commstatus.slave_online[i] = cur.slave_online[i];
    commstatus.slave_operational[i] = cur.slave_operational[i];
  }
  commstatus.master_link = cur.master_link;
  commstatus.master_al_states = cur.master_al_states;
  commstatus.master_slaves_responding = cur.master_slaves_responding;
  commstatus.working_counter = cur.working_counter;
  commstatus.working_counter_state = cur.working_counter_state;

	*/
	
  /* start at (0, 0, 0) */
  if(!odometry_initialized) {
    for (i = 0; i < 4; i++){
      last_odometry_position[i] = position[i];
  	}
    odometry_initialized = 1;
  }

  /* compute differences of encoder readings and convert to meters */
  for (i = 0; i < 4; i++) {
    d_wheel[i] = (int) (position[i] - last_odometry_position[i]) * (1.0/(odometry_constant*odometry_correction));
    /* remember last wheel position */
    last_odometry_position[i] = position[i];
  }

	//Give the difference in wheel position and returns the change in Cartisen
  jac_inverse(d_wheel, d);
    
  ang = odometry[2] + d[2]/2.0;

  odometry[0] += d[0]*cos(ang) - d[1]*sin(ang);
  odometry[1] += d[0]*sin(ang) + d[1]*cos(ang);
  odometry[2] += d[2];

  /* return current odometry values */
  *x = odometry[0];
  *y = odometry[1];
  *a = odometry[2];

  return 0;
}

commstatus_t omnidrive_commstatus()
{
  return commstatus;
}

void omnidrive_status(char *drive0, char *drive1, char *drive2, char *drive3, int *estop)
{
  //drive_status(drive0, 0);
  //drive_status(drive1, 1);
  //drive_status(drive2, 2);
  //drive_status(drive3, 3);

  //if(estop)
  //  *estop = 0x80 & (status[0] & status[1] & status[2] & status[3]);
}

double omnidrive_limit(double x, double l){
	if(x>l) {
		return l;
	}
	
	if(x<-l){
		return -l;
	}
	
	return x;
}



