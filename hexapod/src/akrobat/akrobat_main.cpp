#include <ros/ros.h>

#include <akrobat/akrobat_init.h>
#include <akrobat/Akrobat.h>
#include <akrobat/ControlRandomSampling.h>
#include <sstream>
#include <algorithm>
#include <ctime>
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

using namespace std;
using namespace ros;

int mode; //0 = forward, 1=left, 2 =right, 3= backward, 4=stop
float lin_x, ang_z;
void moveCallback(const geometry_msgs::Twist &twist_msg)
{
	lin_x = twist_msg.linear.x;
	ang_z = twist_msg.angular.z;

	if (lin_x > 0)
	{
		ROS_INFO_STREAM("vorwärts: "
						<< " linear x= " << lin_x << endl);
		mode = 0;
	}
	else if (ang_z > 0)
	{
		ROS_INFO_STREAM("links, "
						<< " angular z= " << ang_z << endl);
		mode = 1;
	}
	else if (ang_z < 0)
	{
		ROS_INFO_STREAM("rechts, "
						<< " angular z= " << ang_z << endl);
		mode = 2;
	}
	else if (lin_x < 0)
	{
		ROS_INFO_STREAM("rückwärts: "
						<< " linear x= " << lin_x << endl);
		mode = 3;
	}
	else if (lin_x == 0 && ang_z == 0)
	{
		ROS_INFO_STREAM("stop movement: " << endl);
		mode = 4;
	}
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "akrobat_main");
	ros::NodeHandle nodeHandle;
	srand(time(NULL));

	Akrobat akrobat;
	ros::Subscriber moveSub = nodeHandle.subscribe("/cmd_vel", 1000, &moveCallback);
	ros::Rate spinRate(20);
	mode = 0;
	ControlRandomSampling controlRandomSampling(akrobat);

	while (ros::ok())
	{
		if (mode == 0)
		{
			controlRandomSampling.forwardMovement();
		}
		else if (mode == 1)
		{
			controlRandomSampling.leftTurn();
		}
		else if (mode == 2)
		{
			controlRandomSampling.rightTurn();
		}
		else if (mode == 3)
		{
			controlRandomSampling.backwardMovement();
		}
		else if (mode == 4)
		{
			controlRandomSampling.stopMovement();
		}

		//Pass movement to akrobat
		akrobat.loadMovements(controlRandomSampling.readMovement());

		akrobat.runAkrobat();

		spinRate.sleep();
		ros::spinOnce();
	}
}
