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
#include <sensor_msgs/JointState.h>

#include <tf/transform_broadcaster.h>

#include <std_msgs/Float64MultiArray.h>

#include <nav_msgs/Odometry.h>

#include "omnilib/omnilib.h"


/*****************************************************************************/
/* 												Class declaration                      						 */
/*****************************************************************************/

class Omnidrive
{
private:
  ros::NodeHandle n_;

  std::string frame_id_;
  std::string child_frame_id_;
  std::string power_name_;

	//void base_state(const sensor_msgs::JointState::ConstPtr& msg);



public:
  Omnidrive();
  void main();
};

	std::vector<double> Pos; 
	int got_pos;
/*****************************************************************************/
/* 												Omni Instantiation                     						 */
/*****************************************************************************/

Omnidrive::Omnidrive() : 
	n_("omni_odometry_publisher")
{
  n_.param("frame_id", frame_id_, std::string("/odom"));
  n_.param("child_frame_id", child_frame_id_, std::string("/base_link"));
  n_.param("power_name", power_name_, std::string("Wheels"));

}

void base_state(const sensor_msgs::JointState::ConstPtr& msg){
    Pos = msg->position;
  	got_pos = 1;

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


	//IO
  ros::Subscriber sub = n_.subscribe("/base_interface/state", 1000, base_state);
	ros::Publisher odom_pub = n_.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster transforms;

  acc_max /= loop_frequency;

  if(omnidrive_init() != 0) {
    ROS_ERROR("failed to initialize omnidrive");
    ROS_ERROR("Check that the base is turned on, if it is ask a supervisor!");
    return;
  }
  // This function is used to add a correction value if the base seems to be slightly off.
  omnidrive_set_correction(drift);


	ros::Time current_time;

  ros::Rate r(loop_frequency);
	printf("Entering ROS main loop\n");
  while(n_.ok()) {
    current_time = ros::Time::now();
		//This make sure you don't get an error for the vector not being set.
		if(got_pos == 1){

			omnidrive_odometry(Pos[0],Pos[1],Pos[2],Pos[3], &x, &y, &a);

			printf("x %f, y %f, a%f\n",x,y,a);
			
			//
			tf::Quaternion q;
			q.setRPY(0, 0, a);
			tf::Transform pose(q, tf::Point(x, y, 0.0));
			transforms.sendTransform(tf::StampedTransform(pose, current_time, frame_id_, child_frame_id_));
			tf_publish_counter = 0;

			//since all odometry is 6DOF we'll need a quaternion created from yaw
    	geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(a);//th);

			//next, we'll publish the odometry message over ROS
	    nav_msgs::Odometry odom;
	    odom.header.stamp = current_time;
	    odom.header.frame_id = frame_id_;
	
	    //set the position
	    odom.pose.pose.position.x = x;
	    odom.pose.pose.position.y = y;
	    odom.pose.pose.position.z = 0.0;
	    odom.pose.pose.orientation = odom_quat;
	
	    //set the velocity
	    odom.child_frame_id = child_frame_id_;
	    odom.twist.twist.linear.x = x;//vx;
	    odom.twist.twist.linear.y = y;//vy;
	    odom.twist.twist.angular.z = 0.0;//vth;
	
	    //publish the message
	    odom_pub.publish(odom);



		}

    ros::spinOnce();
    r.sleep();
  }

}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "omni_base");

	printf("Omni Base started\n");

  Omnidrive drive;
  drive.main();

  return 0;
}
