#include <ros/ros.h>

#include <akrobat/akrobat_init.h>
#include <akrobat/Akrobat.h>
#include <akrobat/ControlRandomSampling.h>
#include <sstream>
#include <algorithm>
#include <ctime>

using namespace std;
using namespace ros;

int main(int argc, char** argv)
{
	init(argc, argv, "akrobat_main");
	
	srand(time(NULL));

	Akrobat akrobat;

	Rate spinRate(20);

	ControlRandomSampling controlRandomSampling(akrobat);
	
	// Generate movement.xml
	controlRandomSampling.randomSampling();
	//controlRandomSampling.tripodGait();

	//Pass movement to akrobat
	akrobat.loadMovements(controlRandomSampling.readMovement());

	while (ros::ok())
	{
		akrobat.runAkrobat();

		ros::spinOnce();
		spinRate.sleep();
	}
}