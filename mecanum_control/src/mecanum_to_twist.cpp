
#include "mecanum_to_twist.h"

#include <stdio.h>
#include "omnilib.h"
#include <vector>
#include <string>
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"

#include "ros/ros.h"

double des_x, des_y, des_a;

void test_twist(double x, double y, double a, double fl, double fr, double bl, double br){
    twist_to_ws(x, y, a, fl, fr, bl, br);
    printf("twist= %4.2f %4.2f %4.2f  wheels= %4.2f %4.2f %4.2f %4.2f\n", x, y, a, fl, fr, bl, br);
}

void twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
	des_x = msg->linear.x;
	des_y = msg->linear.y;
	des_a = msg->angular.z;
}

int main(int argc, char **argv) {

	des_x = 0.0;
	des_y = 0.0;
	des_a = 0.0;


	ros::init(argc, argv, "mecanum_controller");

	ros::NodeHandle n("~");

    double fl, fr, bl, br;


    ros::Subscriber sub = n.subscribe("des_twist", 1000, twistCallback);
    ros::Rate loop_rate(100);
    ros::Publisher mc_pub = n.advertise<sensor_msgs::JointState>("des_wheel_velocities", 1000);


    std::vector<double> des_pos;
    std::vector<double> des_vel;
    std::vector<double> des_eff;
    std::vector<std::string> name;


    name.push_back("FL");
    name.push_back("FR");
    name.push_back("BL");
    name.push_back("BR");

    unsigned int counter = 0;

    while (ros::ok())
    {

    	twist_to_ws(des_x, des_y, des_a, fr, fl, bl, br);
    	des_vel.clear();
    	des_vel.push_back(fr);
    	des_vel.push_back(fl);
    	des_vel.push_back(bl);
    	des_vel.push_back(br);


        counter ++;
        sensor_msgs::JointState js;
        js.header.seq = counter;
        js.header.stamp = ros::Time::now();
        js.header.frame_id = "/base_link";

        js.name = name;
    	js.velocity = des_vel;

    	mc_pub.publish(js);

      	ros::spinOnce();
      	loop_rate.sleep();
    }


    return 0;


}
