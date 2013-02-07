/*
 * Copyright (C) 2009 by Ingo Kresse <kresse@in.tum.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <unistd.h>
#include <sys/time.h>
#include <math.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <soft_runstop/Handler.h>
#include <tf/transform_broadcaster.h>
#include <omni_base_msgs/PowerState.h>
#include <std_msgs/Float64MultiArray.h>

extern "C" {
#include "omnilib/omnilib.h"
}


/*****************************************************************************/
/* 												Class declaration                      						 */
/*****************************************************************************/

class Omnidrive
{
private:
  ros::NodeHandle n_;
  ros::Publisher power_pub_;
  ros::Subscriber power_sub_;
  ros::Time watchdog_time_;
  double drive_[3]; //The desired speed [0] = x; [1] = y;  [2] = z (rotation)
  double drive_last_[3];
  soft_runstop::Handler soft_runstop_handler_;
  std::string frame_id_;
  std::string child_frame_id_;
  std::string power_name_;
  void cmdArrived(const geometry_msgs::Twist::ConstPtr& msg);
  void powerCommand(const omni_base_msgs::PowerState::ConstPtr& msg);
public:
  Omnidrive();
  void main();
};


/*****************************************************************************/
/* 												Omni Instantiation                     						 */
/*****************************************************************************/

Omnidrive::Omnidrive() : 
	n_("omnidrive"), 
	soft_runstop_handler_(Duration(0.5))
{
  n_.param("frame_id", frame_id_, std::string("/odom"));
  n_.param("child_frame_id", child_frame_id_, std::string("/base_link"));
  n_.param("power_name", power_name_, std::string("Wheels"));

	// This pub/sub tells if the wheel has power in for form of a bool
  power_pub_ = n_.advertise<omni_base_msgs::PowerState>("/power_state", 1);
  power_sub_ = n_.subscribe<omni_base_msgs::PowerState>("/power_command", 16, &Omnidrive::powerCommand, this);

	//resets the arrays to zero.
  for(int i=0; i < 3; i++) {
    drive_last_[i] = 0;
    drive_[i] = 0;
  }
	// Sets the init watch dog timer.
  watchdog_time_ = ros::Time::now();
}

void Omnidrive::cmdArrived(const geometry_msgs::Twist::ConstPtr& msg)
{
  // NOTE: No need for synchronization since this is called inside spinOnce() in the main loop

  drive_[0] = msg->linear.x;
  drive_[1] = msg->linear.y;
  drive_[2] = msg->angular.z;

	//Hack maybe should have a separet message.
  if(msg->linear.z == -1) {
    // emergency brake!
    for(int i=0; i < 3; i++) {
      drive_last_[i] = 0;
      drive_[i] = 0;
    }
  }
	//Reset the watch dog.
  watchdog_time_ = ros::Time::now();
}


void Omnidrive::powerCommand(const omni_base_msgs::PowerState::ConstPtr& msg)
{

//There should be a function in here that turns the motors off.

}

void Omnidrive::main()
{
  double speed, acc_max, t, radius, drift;
  int tf_frequency, runstop_frequency;
  const int loop_frequency = 250; // 250Hz update frequency

  n_.param("speed", speed, 100.0); // 0.1
  // default acc: brake from max. speed to 0 within 1.5cm
  n_.param("acceleration", acc_max, 1000.0);  //0.5*speed*speed/0.015
  // radius of the robot
  n_.param("radius", radius, 0.6);
  n_.param("tf_frequency", tf_frequency, 50);
  n_.param("runstop_frequency", runstop_frequency, 10);
  n_.param("watchdog_period", t, 0.5);
  ros::Duration watchdog_period(t);
  n_.param("odometry_correction", drift, 1.0);

  double x=0, y=0, a=0;

  int tf_publish_counter=0;
  int tf_send_rate = loop_frequency / tf_frequency;

  int runstop_publish_counter=0;
  int runstop_send_rate = loop_frequency / runstop_frequency;


  // set acceleration to correct scale
  acc_max /= loop_frequency;

  if(omnidrive_init() != 0) {
    ROS_ERROR("failed to initialize omnidrive");
    ROS_ERROR("Check that the base is turned on, if it is ask a supervisor!");
    return;
  }
  // This function is used to add a correction value if the base seems to be slightly off.
  omnidrive_set_correction(drift);

  tf::TransformBroadcaster transforms;

	//Input from user, which direction and magnitued do we want to drive in.
  ros::Subscriber sub = n_.subscribe("/cmd_vel", 10, &Omnidrive::cmdArrived, this);
  ros::Publisher hard_runstop_pub = n_.advertise<std_msgs::Bool>("/hard_runstop", 1);



  ros::Rate r(loop_frequency);
	printf("Entering ROS main loop\n");
  while(n_.ok()) {

		
    omnidrive_odometry(&x, &y, &a);


		// This makes sure that the desired speed is turn to zero if we do not receave a move command
		// Other wise teh base could crash.
    if(ros::Time::now() - watchdog_time_ > watchdog_period || soft_runstop_handler_.getState()) {
      // emergency brake!
      omnidrive_drive(0, 0, 0);
      
      if((drive_[0] != 0 || drive_[1] != 0 || drive_[2] !=0)
         && !soft_runstop_handler_.getState())
        ROS_WARN("engaged watchdog!");
      for(int i=0; i < 3; i++) {
        drive_last_[i] = 0;
        drive_[i] = 0;
      }

      watchdog_time_ = ros::Time::now();
    }


    //Evil acceleration limitation
    // this runs in a slow loop
    //TODO: Move to the high-speed loop for smoothness
    for(int i=0; i < 3; i++) {
      // acceleration limiting
      double acc = drive_[i] - drive_last_[i];
      double fac_rot = (i == 2) ? 1.0/radius : 1.0;
      acc = omnidrive_limit(acc, acc_max*fac_rot*fac_rot);
      drive_[i] = drive_last_[i] + acc;

      // velocity limiting
      drive_[i] = omnidrive_limit(drive_[i], speed*fac_rot);

      drive_last_[i] = drive_[i];
    }

    omnidrive_drive(drive_[0], drive_[1], drive_[2]);

    // publish odometry readings
    if(++tf_publish_counter == tf_send_rate) {
      tf::Quaternion q;
      q.setRPY(0, 0, a);
      tf::Transform pose(q, tf::Point(x, y, 0.0));
      transforms.sendTransform(tf::StampedTransform(pose, ros::Time::now(), frame_id_, child_frame_id_));
      tf_publish_counter = 0;
    }

    // publish hard runstop state
    if(++runstop_publish_counter == runstop_send_rate) {
      int runstop=0;
      omnidrive_status(0,0,0,0,&runstop);
      std_msgs::Bool msg;
      msg.data = (runstop != 0);
      hard_runstop_pub.publish(msg);
      runstop_publish_counter = 0;
    }


    // process incoming messages
    ros::spinOnce();


    r.sleep();
  }

  omnidrive_shutdown();
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "omni_base");

	printf("Omni Base started\n");

  Omnidrive drive;
  drive.main();

  return 0;
}
