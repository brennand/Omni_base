/*
 * This file is part of the omnimod project.
 *
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
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

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/resource.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include <pthread.h>
#include <stdint.h>

#include "realtime.h"  // defines omniread_t, omniwrite_t



/*****************************************************************************/
	
#define FREQUENCY 1000

/*****************************************************************************/



static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static int exiting = 0;
static pthread_t thread;
static int misses=0;

/*****************************************************************************/

static int max_v = 100;

static omniwrite_t tar, tar_buffer;  /* Target velocities */
static omniread_t cur, cur_buffer;    /* Current velocities/torques/positions */


void cyclic_task()
{

	
	//Talt to the controllers
	
	//send tar
	
	// = tar.target_velocity;
	
	// receave cur
	
	//  cur.position = ;
	//  cur.actual_velocity = ;

printf(" :-) \n");

}


/*****************************************************************************/


static void enforce_max_velocities(omniwrite_t *t)
{
	int i;

	for (i = 0; i < 2; i++) {
		t->target_velocity[i] =
		  (t->target_velocity[i] > max_v) ? max_v : t->target_velocity[i];
		t->target_velocity[i] =
		  (t->target_velocity[i] < -max_v) ? -max_v : t->target_velocity[i];
	}
}


/*****************************************************************************/

static void stop_motors(void)
{
	//Some command to trun the motors off.
}


static void timespecInc(struct timespec *tick, int nsec)
{
  tick->tv_nsec += nsec;
  while (tick->tv_nsec >= 1e9)
  {
    tick->tv_nsec -= 1e9;
    tick->tv_sec++;
  }
}

//This is the thread that will run with high priority
void* realtimeMain(void* udata)
{
  struct timespec tick;
  int period = 1e+6; // 1 ms in nanoseconds

  while(!exiting)
  {
    cyclic_task(); // Send and receave from the motor controllers

    if(pthread_mutex_trylock(&mutex) == 0)
    {
      tar = tar_buffer;
      cur_buffer = cur;
      pthread_mutex_unlock(&mutex);
    }

    // Compute end of next period
    timespecInc(&tick, period);

    struct timespec before;
    clock_gettime(CLOCK_REALTIME, &before);
    if ((before.tv_sec + before.tv_nsec/1e9) > (tick.tv_sec + tick.tv_nsec/1e9))
    {
      // We overran, snap to next "period"
      tick.tv_sec = before.tv_sec;
      tick.tv_nsec = (before.tv_nsec / period) * period;
      timespecInc(&tick, period);

      misses++;
    }
    // Sleep until end of period
    clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);

  }

  stop_motors();



  return 0;
}


/*****************************************************************************/

/* Interface functions */

int start_omni_realtime(int max_vel)
{
	max_v = max_vel;

	printf("Init omni...\n");

	/* Zero out the target/current data structs, just in case. */
	memset(&tar, 0, sizeof(tar));
	memset(&cur, 0, sizeof(cur));
	cur.magic_version = OMNICOM_MAGIC_VERSION;

	printf("Starting omni....\n");


	printf("Starting cyclic thread.\n");

    pthread_attr_t tattr;
    struct sched_param sparam;
    sparam.sched_priority = sched_get_priority_max(SCHED_FIFO);
    pthread_attr_init(&tattr);
    pthread_attr_setschedpolicy(&tattr, SCHED_FIFO);
    pthread_attr_setschedparam(&tattr, &sparam);
    pthread_attr_setinheritsched (&tattr, PTHREAD_EXPLICIT_SCHED);
    
    if(pthread_create(&thread, &tattr, &realtimeMain, 0) != 0) {
      printf("# ERROR: could not create realtime thread\n");
      return 0;
    }


	printf(" Realtime thread Started.\n");

	return 1;
}

/*****************************************************************************/

void stop_omni_realtime(void)
{
	printf("Stopping...\n");


	/* Signal a stop the realtime thread */
    exiting = 1;
    pthread_join(thread, 0);

	/* Now stop all motors. */
	stop_motors();

	printf("Unloading.\n");
}


void omni_write_data(struct omniwrite data)
{
  pthread_mutex_lock(&mutex);
  tar_buffer = data;
  enforce_max_velocities(&tar_buffer);
  pthread_mutex_unlock(&mutex);
}

struct omniread omni_read_data()
{
  struct omniread data;
  pthread_mutex_lock(&mutex);
  data = cur_buffer;
  pthread_mutex_unlock(&mutex);
  return data;
}
