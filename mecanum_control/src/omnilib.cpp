

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


void twist_to_ws(double x, double y, double a, double &fl, double &fr, double &bl, double &br)
{
	//This function converst the twist to the wheel velocities
	// It also checks for some limits

	// speed limits for the robot
	double wheel_limit = 1.0 ;//0.8;  // a single wheel may drive this fast (m/s)
	double cart_limit = 0.5 ; //0.5;   // any point on the robot may move this fast (m/s)
	double radius = 0.7;       // (maximum) radius of the robot (m)

	double corr_wheels, corr_cart, corr;

	double cartesian_speeds[3] = {x, y, a};
	double wheel_speeds[4];
	int i;

	// check for limits

	// cartesian limit: add linear and angular parts
	corr_cart = cart_limit / (sqrt(x*x + y*y) + radius*fabs(a));

	// wheel limit: for one wheel, x,y and a always add up
	corr_wheels = wheel_limit / (fabs(x) + fabs(y) + fabs(a));

	// get limiting factor as min(1, corr_cart, corr_wheels)
	corr = (1 < corr_cart) ? 1 : ((corr_cart < corr_wheels) ? corr_cart : corr_wheels);

	jac_forward(cartesian_speeds, wheel_speeds);

	fl = wheel_speeds[0];
	fr = wheel_speeds[1];
	bl = wheel_speeds[2];
	br = wheel_speeds[3];



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



