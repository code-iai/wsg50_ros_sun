/*
    ROS node to control the normal force

    Copyright 2018 Università della Campania Luigi Vanvitelli

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "ros/ros.h"
#include <signal.h>

//#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <std_msgs/Float32.h>


ros::NodeHandle * nh;
ros::Publisher velPub;

double fz = 0.0;
double fr = 0.0;
float vel_cmd = 0.0;
float control_gain = 1.0;
std_msgs::Float32 velMsg;
void readForces(const geometry_msgs::WrenchStamped::ConstPtr& forceMsg)
{
	fz = forceMsg->wrench.force.z;

	vel_cmd = -control_gain*(fr-fabs(fz));

	velMsg.data = vel_cmd;

	velPub.publish(velMsg);
}

void readCommand(const std_msgs::Float32::ConstPtr& forceMsg)
{
	fr = fabs(forceMsg->data);
}

void intHandler(int dummy) {
    velMsg.data = 0.0;
    velPub.publish(velMsg);
    velPub.publish(velMsg);
    velPub.publish(velMsg);
    velPub.publish(velMsg);
    ros::shutdown();
}


int main(int argc, char *argv[])
{
    
    ros::init(argc, argv, "force_controller");

    nh = new ros::NodeHandle();

	// Publisher
	//ros::Subscriber force_sub = nh->subscribe("/rft_data", 1, readForces);
	ros::Subscriber force_sub = nh->subscribe("/wsg_50_driver_sun/finger0/wrench", 1, readForces);
	ros::Subscriber force_command_sub = nh->subscribe("/wsg_50_driver_sun/commandForce", 1, readCommand);
	velPub = nh->advertise<std_msgs::Float32>( "/wsg_50_driver_sun/goal_speed",1);

    nh->param("controlGain" , control_gain, (float)1.0 );

    control_gain = fabs(control_gain);

    signal(SIGINT, intHandler);
    ros::spin();

	velMsg.data = 0.0;
	velPub.publish(velMsg);
	velPub.publish(velMsg);
	velPub.publish(velMsg);
	velPub.publish(velMsg);


    return 0;
}
