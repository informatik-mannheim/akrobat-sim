#include <akrobat/LegSetting.h>
#include <tf/tf.h>

LegSetting::LegSetting()
{
}

LegSetting::LegSetting(double rotationOfCoxa, double minCoxa, double minFemur, double minTibia, double maxCoxa, double maxFemur, double maxTibia, tf::Vector3 trajectory) :
	rotationOfCoxa(rotationOfCoxa),
	minCoxa(minCoxa),
	minFemur(minFemur),
	minTibia(minTibia),
	maxCoxa(maxCoxa),
	maxFemur(maxFemur),
	maxTibia(maxTibia),
	trajectory(trajectory)
{
}