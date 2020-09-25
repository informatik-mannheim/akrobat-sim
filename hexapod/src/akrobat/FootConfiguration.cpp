#include "akrobat/FootConfiguration.h"

FootConfiguration::FootConfiguration(const signed char state, const Position& position)
    : state(state), currentPosition(position)
{}

FootConfiguration FootConfiguration::deepCopy()
{
    FootConfiguration::Position newPosition;

    for(int i = 0; i < FootConfiguration::FOOT_SIZE; ++i)
    {
        newPosition[i].position.setX(currentPosition[i].position.x());
        newPosition[i].position.setY(currentPosition[i].position.y());
        newPosition[i].position.setZ(currentPosition[i].position.z());

        newPosition[i].initPosition.setX(currentPosition[i].initPosition.x());
        newPosition[i].initPosition.setY(currentPosition[i].initPosition.y());
        newPosition[i].initPosition.setZ(currentPosition[i].initPosition.z());

        newPosition[i].delta.setX(currentPosition[i].delta.x());
        newPosition[i].delta.setY(currentPosition[i].delta.y());
        newPosition[i].delta.setZ(currentPosition[i].delta.z());
    }

    return FootConfiguration(state, newPosition);
}

signed char FootConfiguration::FOOT_BIT[] =
{
    FOOT_EL_1,
    FOOT_EL_2,
    FOOT_EL_3,
    FOOT_EL_4,
    FOOT_EL_5,
    FOOT_EL_6
};

std::vector<signed char> FootConfiguration::validStates =
{
	FOOT_EL_1 | FOOT_EL_4 | FOOT_EL_5,
	FOOT_EL_2 | FOOT_EL_3 | FOOT_EL_6,

	FOOT_EL_1 | FOOT_EL_2 | FOOT_EL_4 | FOOT_EL_5,
	FOOT_EL_1 | FOOT_EL_3 | FOOT_EL_4 | FOOT_EL_5,
	FOOT_EL_1 | FOOT_EL_4 | FOOT_EL_5 | FOOT_EL_6,
	FOOT_EL_1 | FOOT_EL_2 | FOOT_EL_3 | FOOT_EL_6,
	FOOT_EL_2 | FOOT_EL_3 | FOOT_EL_4 | FOOT_EL_6,
	FOOT_EL_2 | FOOT_EL_3 | FOOT_EL_5 | FOOT_EL_6,
	FOOT_EL_1 | FOOT_EL_2 | FOOT_EL_5 | FOOT_EL_6,

	FOOT_EL_1 | FOOT_EL_2 | FOOT_EL_3 | FOOT_EL_4 | FOOT_EL_5,
	FOOT_EL_1 | FOOT_EL_2 | FOOT_EL_4 | FOOT_EL_5 | FOOT_EL_6,
	FOOT_EL_1 | FOOT_EL_3 | FOOT_EL_4 | FOOT_EL_5 | FOOT_EL_6,
	FOOT_EL_1 | FOOT_EL_2 | FOOT_EL_3 | FOOT_EL_4 | FOOT_EL_6,
	FOOT_EL_1 | FOOT_EL_2 | FOOT_EL_3 | FOOT_EL_5 | FOOT_EL_6,
	FOOT_EL_2 | FOOT_EL_3 | FOOT_EL_4 | FOOT_EL_5 | FOOT_EL_6,

	FOOT_EL_1 | FOOT_EL_2 | FOOT_EL_3 | FOOT_EL_4 | FOOT_EL_5 | FOOT_EL_6
};

std::vector<std::vector<signed char>> FootConfiguration::transitions;

void FootConfiguration::init() {
    int max = 64;

    FootConfiguration::transitions.resize(max);

    bool valid[max] = { false };

    for(int state : FootConfiguration::validStates) 
    {
        valid[state] = true;
    }

    for(int source = 0; source < max; source++)
    {
        if(!valid[source]) {
            continue;
        }

        for(int dest = 0;  dest < max; dest++)
        {
            if(!valid[dest]) {
                continue;
            }

            for(int foot = 0; foot < FootConfiguration::FOOT_SIZE; foot++)
            {
                if((source ^ dest ) == FootConfiguration::FOOT_BIT[foot]) {
                    FootConfiguration::transitions.at(source).push_back(dest);
                }
            }
        }
    }
}

FootConfiguration::Position FootConfiguration::get() const {
    return currentPosition;
}