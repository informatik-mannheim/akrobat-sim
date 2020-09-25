#ifndef LEGSETTING_H
#define LEGSETTING_H

#include <tf/tf.h>

class LegSetting
{
public:
	double rotationOfCoxa = 0.0;

	// min limit of coxa joint initialization
	double minCoxa = 0.0; // [�] (coxa joint) alpha angle min limit
	double minFemur = 0.0; // [�] (femur joint) beta angle min limit
	double minTibia = 0.0; // [�] (tibia joint) gamma angle min limit

	// max limit of coxa jointinitialization
	double maxCoxa = 0.0; // [�] (coxa joint) alpha angle max limit
	double maxFemur = 0.0; // [�] (femur joint) beta angle max limit
	double maxTibia = 0.0; // [�] (tibia joint) gamma angle max limit
	
	tf::Vector3 trajectory = tf::Vector3(0.0, 0.0, 0.0);

	LegSetting();
	LegSetting(double rotationOfCoxa, double minCoxa, double minFemur, double minTibia, double maxCoxa, double maxFemur, double maxTibia, tf::Vector3 trajectory);
};

#endif // LEGSETTING_H