#include <akrobat/Akrobat.h>

#define _USE_MATH_DEFINES
#include <cmath>

#include <fstream>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <angles/angles.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

#include <akrobat/akrobat_init.h>

using namespace std;
using namespace tf;
using namespace ros;
using namespace angles;

Akrobat::Akrobat() :
	LENGTH_COXA(0.070),
	LENGTH_FEMUR(0.096),
	LENGTH_TIBIA(0.162)
{
	legSettings[LEFT_FRONT] = LegSetting(0.0, -26.0, -99.0, -135.0, 65.0, 96.0, 135.0, tf::Vector3(0.0, 0.0, 0.0));
	legSettings[RIGHT_FRONT] = LegSetting(180.0, -71.0, -99.0, -135.0, 28.0, 96.0, 135.0, tf::Vector3(0.0, 0.0, 0.0));
	legSettings[LEFT_MIDDLE] = LegSetting(0.0, -51.0, -99.0, -135.0, 48.0, 96.0, 135.0, tf::Vector3(0.0, 0.0, 0.0));
	legSettings[RIGHT_MIDDLE] = LegSetting(180.0, -51.0, -99.0, -135.0, 48.0, 96.0, 135.0, tf::Vector3(0.0, 0.0, 0.0));
	legSettings[LEFT_REAR] = LegSetting(0.0, -71.0, -99.0, -135.0, 30.0, 96.0, 135.0, tf::Vector3(0.0, 0.0, 0.0));
	legSettings[RIGHT_REAR] = LegSetting(180.0, -26.0, -99.0, -135.0, 75.0, 96.0, 135.0, tf::Vector3(0.0, 0.0, 0.0));

	jointPub = n.advertise<sensor_msgs::JointState>("/goal_joint_states", 1);

	js.name.resize(18);
	js.position.resize(18);
	js.velocity.resize(18);
	js.effort.resize(18);

	for (int i = 0, j = 1, k = 1; i < 18; i++)
	{
		string name = "m";
		name += ('0' + j);
		name += ('0' + k++);

		js.name[i] = name;
		js.velocity[i] = 0.0;
		js.effort[i] = 0.0;

		if (k > 3)
		{
			j++;
			k = 1;
		}
	}
}

void Akrobat::loadMovements(std::vector<std::vector<tf::Vector3>> movements) {
	Akrobat::movements = movements;
}
	
tf::Vector3 Akrobat::getTrajectory(int legNumber) const {
	if(movementPosition > -1 && movementPosition < movements[legNumber].size()) {
		tf::Vector3 currentTrajectory = movements[legNumber][movementPosition];
		tf::Vector3 previousTrajectory = Vector3(0.0, 0.0, 0.0);
		
		if(movementPosition > 0) {
			previousTrajectory = movements[legNumber].at(movementPosition - 1);
		}
		
		tf::Vector3 difference = currentTrajectory - previousTrajectory;
		
		difference.setX(difference.x() * (-0.5 * cos(M_PI * currentTick / maxTicks) + 0.5));
		difference.setY(difference.y() * (-0.5 * cos(M_PI * currentTick / maxTicks) + 0.5));
		difference.setZ(difference.z() * (-0.5 * cos(M_PI * currentTick / maxTicks) + 0.5));
		
		return previousTrajectory + difference;
	}
	
	return tf::Vector3(9999.0,0,0);
}
	
void Akrobat::runAkrobat()
{
	js.header.stamp = ros::Time::now();
	
	for(int i = 0; i < 6; i++) {
		// Setup default position
		tf::Quaternion rotOfCoxa;
		tf::Quaternion rotHexapodToCoxa;
		tf::Quaternion rotCoxaToFemur;
		tf::Quaternion rotFemurToTibia;
		tf::Vector3 transCoxaToFemur;
		tf::Vector3 transTrajectory;
		tf::Vector3 trajectory = Akrobat::getTrajectory(i);

		bool isTrajectoryEmpty = trajectory.x() == 9999.0;

		if(isTrajectoryEmpty && movementPosition > -1) {
			std::cout << "recognized as done, skip" << std::endl;
			continue;
		}
		else if(isTrajectoryEmpty) {
			std::cout << " recognized as stand up stage" << std::endl;
			trajectory = tf::Vector3(0,0,0);
		}

		rotOfCoxa.setRPY(0.0, 0.0, from_degrees(legSettings[i].rotationOfCoxa));		
		rotHexapodToCoxa.setRPY(0.0, 0.0, 0.0);		
				
		if(i == 0 || i == 2 || i == 4) {
			transCoxaToFemur = tf::Vector3(LENGTH_COXA, 0.0, 0.0);
			rotCoxaToFemur.setRPY(3.14, -0.174533, 0.0);
			
			transTrajectory = tf::Vector3(
				trajectory.x(),
				trajectory.y(),
				trajectory.z()
			);
		}
		else {			
			transCoxaToFemur = tf::Vector3(-LENGTH_COXA, 0.0, 0.0);
			rotCoxaToFemur.setRPY(0.0, 3.31613, 0.0);
			
			transTrajectory = tf::Vector3(
				-trajectory.x(),
				-trajectory.y(),
				trajectory.z()
			);
		}
		
		rotFemurToTibia.setRPY(0.0, -3.14, 0.0);
		
		tf::Transform transform = (
			tf::Transform(rotOfCoxa, tf::Vector3(0.0, 0.0, 0.0)) *
			(
				tf::Transform(rotHexapodToCoxa, tf::Vector3(0.0, 0.0, 0.0)) *
				tf::Transform(rotCoxaToFemur, transCoxaToFemur) *
				tf::Transform(rotFemurToTibia, tf::Vector3(LENGTH_FEMUR, 0.0, 0.0)) *
				tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, -LENGTH_TIBIA)) * 
				tf::Transform(tf::Quaternion(0, 0, 0, 1), transTrajectory)
			)
		);
		
		// Calculate angels by using the transform
		std::vector<double> angles = Akrobat::inverseKinematics(
			tf::Vector3(
				transform.getOrigin().x(),
				transform.getOrigin().y(),
				transform.getOrigin().z()
			)
		);
		
		// Set the calculated angels to the leg
		Akrobat::moveLeg(i, angles);
	}
	
	if(movementPosition < 0) {
		movementPosition++;
	}
	else {
		cout << currentTick << endl;
		
		currentTick++;
		
		if(currentTick > maxTicks) {
			currentTick = 1;
			movementPosition++;
		}
	}
	
	jointPub.publish(js);
}

std::vector<double> Akrobat::inverseKinematics(const tf::Vector3& point) const
{
	float R, L, ALPHA, BETA1, BETA2, BETA, GAMMA;

	double x = point.x();
	double y = point.y();
	double z = point.z();

	R = sqrt(pow(y, 2) + pow(x, 2));
	ALPHA = atan2(y, x);
	L = sqrt(pow(R - LENGTH_COXA, 2) + pow(z, 2));
	BETA1 = atan2(z, (R - LENGTH_COXA));
	BETA2 = acos(0.5 * (pow(LENGTH_FEMUR, 2) + pow(L, 2) - pow(LENGTH_TIBIA, 2)) / (L * LENGTH_FEMUR));

	GAMMA = acos(0.5 * (pow(LENGTH_TIBIA, 2) + pow(LENGTH_FEMUR, 2) - pow(L, 2)) / (LENGTH_FEMUR * LENGTH_TIBIA));
	BETA = BETA1 + BETA2;
	GAMMA = GAMMA - from_degrees(180);

	std::vector<double> angles = {
		to_degrees(ALPHA),
		to_degrees(BETA),
		to_degrees(GAMMA)
	};
	
	return angles;
}

int Akrobat::moveLeg(int legNum, std::vector<double> angles)
{
	if(!Akrobat::anglesValid(legNum, angles)) {
		return 0;
	}

	js.position[legNum * 3 + 0] = from_degrees(angles[0]);
	js.position[legNum * 3 + 1] = from_degrees(angles[1]);
	js.position[legNum * 3 + 2] = from_degrees(angles[2]);

	return 1;
}


bool Akrobat::IsWithinLimits(const double& value, const double& min, const double& max)
{
	return value >= min && value <= max;
}

bool Akrobat::anglesValid(int legNum, std::vector<double> angles) const
{
	if (!IsWithinLimits(angles[0], legSettings[legNum].minCoxa, legSettings[legNum].maxCoxa))
	{
		cout << "[WARNING] " << "LEG " << legNum << ": angle range of coxa("
			<< legSettings[legNum].minCoxa << ":" << legSettings[legNum].maxCoxa << ") joint is exceeded " << angles[0] << endl;
		return false;
	}

	if (!IsWithinLimits(angles[1], legSettings[legNum].minFemur, legSettings[legNum].maxFemur))
	{
		cout << "[WARNING] " << "LEG " << legNum << ": angle range of femur("
			<< legSettings[legNum].minFemur << ":" << legSettings[legNum].maxFemur << ") joint is exceeded " << angles[1] << endl;
		return false;
	}

	if (!IsWithinLimits(angles[2], legSettings[legNum].minTibia, legSettings[legNum].maxTibia))
	{
		cout << "[WARNING] " << "LEG " << legNum << ": angle range of tibia("
			<< legSettings[legNum].minTibia << ":" << legSettings[legNum].maxTibia << ") joint is exceeded " << angles[2] << endl;
		return false;
	}

	return true;
}