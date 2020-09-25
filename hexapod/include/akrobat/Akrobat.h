#ifndef AKROBAT_H
#define AKROBAT_H

#include <akrobat/akrobat_init.h>

#include <string>

#include <ros/ros.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <sensor_msgs/JointState.h>

#include <akrobat/LegSetting.h>

class Akrobat
{
public:
	enum LegNumber : int
	{
		LEFT_FRONT = 0,
		RIGHT_FRONT = 1,
		LEFT_MIDDLE = 2,
		RIGHT_MIDDLE = 3,
		LEFT_REAR = 4,
		RIGHT_REAR = 5
	};

	double LENGTH_COXA;
	double LENGTH_FEMUR;
	double LENGTH_TIBIA;
	
	std::vector<std::vector<tf::Vector3>> movements = {
		{},
		{},
		{},
		{},
		{},
		{}
	};
	
	int movementPosition = -10;
	int currentTick = 1;
	int maxTicks = 8;

	LegSetting legSettings[numberOfLegs];
	
	sensor_msgs::JointState js;

	tf::TransformListener listener;
	tf::TransformBroadcaster broadcaster;

	// constructor
	Akrobat();

	// execute important fuction to run the hexapod
	void runAkrobat();

	// calculate the inverse kinematics for each leg
	std::vector<double> inverseKinematics(const tf::Vector3& point) const;
	
	// move the leg to target position
	int moveLeg(int legNum, std::vector<double> angles);

	tf::Vector3 getTrajectory(int legNumber) const;

	void loadMovements(std::vector<std::vector<tf::Vector3>> movements);

	bool anglesValid(int legNum, std::vector<double> angles) const;
	static bool IsWithinLimits(const double& value, const double& min, const double& max);
	
	ros::Publisher jointPub;

private:
	ros::NodeHandle n;
};

#endif