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

void omnidrive_set_correction(double drift)
{
  odometry_correction = drift;
}

int omnidrive_odometry(double Vel1,double Vel2,double Vel3,double Vel4,double Pos1,double Pos2,double Pos3,double Pos4,double *x, double *y, double *a)
{

//  omniread_t cur; // Current velocities / torques / positions 
//  int i;
  double d_wheel[4], d[3], ang;

	double position[4];
	position[0] = Pos1;
	position[1] = Pos2;	
	position[2] = Pos3;
	position[3] = Pos4;
	
	d_wheel[0] = Vel1;
	d_wheel[1] = Vel2;
	d_wheel[2] = Vel3;
	d_wheel[3] = Vel4;

	
  /* start at (0, 0, 0) */
 
 /*
  if(!odometry_initialized) {
    for (i = 0; i < 4; i++){
      last_odometry_position[i] = position[i];
  	}
    odometry_initialized = 1;
  }
	*/
  /* compute differences of encoder readings and convert to meters */
//  for (i = 0; i < 4; i++) {
  //  d_wheel[i] = (int) (position[i] - last_odometry_position[i]) * (1.0/(odometry_constant*odometry_correction));
    /* remember last wheel position */
  //  last_odometry_position[i] = position[i];
//  }

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


double omnidrive_limit(double x, double l){
	if(x>l) {
		return l;
	}
	
	if(x<-l){
		return -l;
	}
	
	return x;
}



