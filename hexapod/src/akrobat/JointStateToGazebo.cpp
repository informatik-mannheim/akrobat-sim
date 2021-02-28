#include <vector>
#include <string>
#include "ros/ros.h"
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>

using namespace std;
using namespace ros;

NodeHandle* nodeHandle;

string gazeboPrefix;
vector<Publisher> publishers;

void goalJointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
	std_msgs::Float64 message;

	if (publishers.size() == 0)
	{
		for (int i = 0; i < msg->name.size(); i++)
		{
			publishers.push_back(nodeHandle->advertise<std_msgs::Float64>(gazeboPrefix + msg->name[i] + "_position_controller/command", 1)); //Gazebo hört auf _position_controller_command
		}
	}

	for (int i = 0; i < msg->position.size(); i++)
	{
		message.data = msg->position[i];

		publishers[i].publish(message);
	}
}

int main(int argc, char** argv)
{
	init(argc, argv, "JointStateToGazebo");

	nodeHandle = new NodeHandle();

	int publish_frequency = 20;
	string goalNodeName;

	Subscriber subscriber;

	nodeHandle->param<std::string>("goalNodeName", goalNodeName, "/goal_joint_states");
	nodeHandle->param<std::string>("gazeboPrefix", gazeboPrefix, "/hexapod/");
	nodeHandle->param<int>("publish_frequency", publish_frequency, 20);

	subscriber = nodeHandle->subscribe(goalNodeName, 1, goalJointStateCallback);

	Rate spinRate(publish_frequency);

	// ros main loop
	while (ok())
	{
		spinOnce();

		spinRate.sleep();
	}

	delete nodeHandle;
}