#include <akrobat/ControlRandomSampling.h>

#include <iostream>
#include <string>
#include <vector>
#include <random>
#include <time.h>
#include <cmath>
#include <assert.h>

#include <ros/ros.h>

#include <tf/tf.h>
#include <tf/transform_datatypes.h>

#include <limits>
#include <fstream>

#include "pugixml/pugixml.hpp"

#include "akrobat/FootConfiguration.h"

//test includes▼
#include <akrobat/Trajectory.h>
#include "matplotlib-cpp/matplotlibcpp.h"
namespace plt = matplotlibcpp;

//end test includes

using namespace std;
using namespace ros;

ControlRandomSampling::ControlRandomSampling(Akrobat& akrobat): 
  akrobat(akrobat), 
  validCenterIdx(0), 
  startCenterIdx(0), 
  endCenterIdx(0),
  validCenterPoint(tf::Vector3(0, 0, 0)),
  startCenterPoint(tf::Vector3(0, 0, 0)),
  endCenterPoint(tf::Vector3(0, 0, 0))
{}
  
void ControlRandomSampling::tripodGait() const {
  Trajectory traData = Trajectory();

  traData.caseStep[0] = 2;
  traData.caseStep[1] = 1;
  traData.caseStep[2] = 1;
  traData.caseStep[3] = 2;
  traData.caseStep[4] = 2;
  traData.caseStep[5] = 1;

  traData.initAmpX = 0.040;
  traData.initAmpY = 0.040;
  traData.initAmpZ = 0.040;
  traData.tick = 0;
  
  traData.ampX[0] = traData.initAmpX * 0;
  traData.ampX[1] = traData.initAmpX * 0;
  traData.ampY[0] = traData.initAmpY;
  traData.ampY[1] = traData.initAmpY;
  traData.ampZ[0] = traData.initAmpZ;
  traData.ampZ[1] = traData.initAmpZ;
  traData.ampX[2] = traData.initAmpX * 0;
  traData.ampX[3] = traData.initAmpX * 0;
  traData.ampY[2] = traData.initAmpY;
  traData.ampY[3] = traData.initAmpY;
  traData.ampZ[2] = traData.initAmpZ;
  traData.ampZ[3] = traData.initAmpZ;
  traData.ampX[4] = traData.initAmpX * 0;
  traData.ampX[5] = traData.initAmpX * 0;
  traData.ampY[4] = traData.initAmpY;
  traData.ampY[5] = traData.initAmpY;
  traData.ampZ[4] = traData.initAmpZ;
  traData.ampZ[5] = traData.initAmpZ;
  
  std::vector<std::vector<tf::Vector3>> footPoints = {{}, {}, {}, {}, {}, {}};
  
  for(int step = 0; step < 100; step++)
  {    
    for(int legNum = 0; legNum < 6; legNum++)
    {
      float x;
      float y;
      float z;
      
      switch (traData.caseStep[legNum])
      {
        case 1:
          x = -traData.ampX[legNum];
          y = -traData.ampY[legNum];
          z = abs(traData.ampZ[legNum]);
          
          traData.caseStep[legNum] = 2;
          break;

        case 2:
          x = traData.ampX[legNum] - 0.002 * traData.ampX[legNum];
          y = traData.ampY[legNum] - 0.002 * traData.ampY[legNum];
          z = 0;
          traData.caseStep[legNum] = 1;
          break;
      }
      
      footPoints.at(legNum).push_back(tf::Vector3(x, y, z));
    }
    
    traData.tick++;
    if (traData.tick > 15 - 1)
    {
      traData.tick = 0;
    }
  }
  
  ControlRandomSampling::exportMovement(footPoints);
}

std::vector<std::vector<tf::Vector3>> ControlRandomSampling::readMovement() const {
  std::vector<std::vector<tf::Vector3>> footPoints = {{}, {}, {}, {}, {}, {}};
  
  pugi::xml_document doc;
  
  if (!doc.load_file("movements.xml"))
  {
    cout << "File not found" << endl;
  }
  
  pugi::xml_node movement = doc.child("movement");
   
  for (pugi::xml_node foot : movement.children("foot")) 
  {
    int footIndex = std::stoi(foot.attributes().begin()->value());
    
    for (pugi::xml_node step : foot.children("step"))
    {
      tf::Vector3 point = tf::Vector3(
        std::stof(step.child_value("x")),
        std::stof(step.child_value("y")),
        std::stof(step.child_value("z"))
      );
      
      footPoints.at(footIndex).push_back(point);
    }
  }
  
  return footPoints;
}

void ControlRandomSampling::exportMovement(std::vector<std::vector<tf::Vector3>>& footPoints) const {
  // Writing start and end points to a movement file
  ofstream movementFile;
  movementFile.open("movements.xml");
    
  movementFile << "<?xml version=\"1.0\"?>" << endl;
  movementFile << "<movement>" << endl;
  
  for(size_t i = 0; i < footPoints.size(); i++) {
    movementFile << "\t<foot number=\"" << i << "\">" << endl;
    for(size_t j = 0; j < footPoints.at(i).size(); j++) {
      movementFile << "\t\t<step>" << endl;
      movementFile << "\t\t\t<x>" << footPoints.at(i).at(j).x() << "</x>" << endl;
      movementFile << "\t\t\t<y>" << footPoints.at(i).at(j).y() << "</y>" << endl;
      movementFile << "\t\t\t<z>" << footPoints.at(i).at(j).z() << "</z>" << endl;
      movementFile << "\t\t</step>" << endl;
    }
    movementFile << "\t</foot>" << endl;
  }
  
  movementFile << "</movement>" << endl;
  
  movementFile.close();
}

void ControlRandomSampling::randomSampling() {
  bool valid;
  int tries = 0;

  do {
    tries++;
    cout << "Try again" << tries << endl;
    valid = generateRandomSampling();
  }
  while(!valid);

  cout << "Finished after " << tries << " tries" << endl;

}

bool ControlRandomSampling::generateRandomSampling() {
  this->validCenterIdx = 0;
  this->startCenterIdx = 0;
  this->endCenterIdx = 0;

  this->validCenterPoint = tf::Vector3(0, 0, 0);
  this->startCenterPoint = tf::Vector3(0, 0, 0);
  this->endCenterPoint = tf::Vector3(0, 0, 0);

  std::vector<Event> events;

  tf::Vector3 start = tf::Vector3(0.0, 0.0, 0.0);
  tf::Vector3 end = tf::Vector3(0.0, 1, 0.0);//tf::Vector3(0.0, 7, 0.0);
  
  FootConfiguration footConfiguration(63, {
    //foot positions are always from coxa
    FootConfiguration::Foot(tf::Vector3(-0.187, 0.068, -0.137), tf::Vector3(-0.187, 0.068, -0.137), tf::Vector3(-0.051, 0.217, 0.0)),
    FootConfiguration::Foot(tf::Vector3(0.187, 0.068, -0.137), tf::Vector3(0.187, 0.068, -0.137), tf::Vector3(0.051, 0.217, 0.0)),
    FootConfiguration::Foot(tf::Vector3(-0.199, 0.0, -0.137), tf::Vector3(-0.199, 0.0, -0.137), tf::Vector3(-0.051, 0.0, 0.0)),
    FootConfiguration::Foot(tf::Vector3(0.199, 0.00, -0.137), tf::Vector3(0.199, 0.00, -0.137), tf::Vector3(0.051, 0.0, 0.0)),
    FootConfiguration::Foot(tf::Vector3(-0.187, -0.068, -0.137), tf::Vector3(-0.187, -0.068, -0.137), tf::Vector3(-0.051, -0.217, 0.0)),
    FootConfiguration::Foot(tf::Vector3(0.187, -0.068, -0.137), tf::Vector3(0.187, -0.068, -0.137), tf::Vector3(0.051, -0.217, 0.0))
  });

  // 1. Generate center points from start to end
  std::vector<tf::Vector3> centerList = ControlRandomSampling::generateCenterPoints(start, end);
  //todo add footPoints --> maybe call it footList and centerList
    
  // 2. Calculate first center range
  ControlRandomSampling::calcCenterRange(
    centerList,
    footConfiguration
  );

  // 3. Push first starting event
  events.push_back(
    Event(
      footConfiguration.deepCopy(),
      this->startCenterPoint + tf::Vector3(0, 0, 0),
      this->endCenterPoint + tf::Vector3(0, 0, 0)
    )
  );

  //Init foot
  FootConfiguration::init();

  int bonus[6] = {3, 3, 3, 3, 3, 3};

  //Init loop values
  double cPos = 0.0;

  int currentStep = 0;
  int maximumSteps = 600;
  //int maximumSteps = 24;

  do {
    //Create break for maximum step number
    currentStep++;

    if(currentStep > maximumSteps) {
      cout << "Maximum number of steps reached" << endl;
      break;
      //return false; //comment in later, bc right now we want to test
    }

    cout << "Loop" << endl;

    //Find out which foot transitions would not violate the stability rules
    std::vector<signed char> validConfigurations;
    std::vector<int> bonuses;

    //Find all valid configurations, their bonuses
    for(signed char configuration : FootConfiguration::transitions[footConfiguration.state])
    {
      //If a feet gets lifted with this new configuration, check violation
      if (footConfiguration.state > configuration)
      {
        FootConfiguration nextConfiguration = footConfiguration.deepCopy();
        nextConfiguration.state = configuration;

        std::vector<tf::Vector3> candidates;
        std::vector<int> candIdx;

        this->testCenterRegion(nextConfiguration, candidates, candIdx, centerList);

        if(candidates.size() == 0) {
          continue;
        }
      }

      validConfigurations.push_back(configuration);
      bonuses.push_back(bonus[whichFoot(footConfiguration.state, configuration)]);
    }

    assert(validConfigurations.size() > 0);

    //Find the bonus sum
    int sumBonus = 0;

    for(signed char bonus : bonuses)
    {
        sumBonus += bonus * bonus * bonus;
    }

    //Find the winner configuration
    signed char winner = -1;

    int r_wheel = (int) (((double) sumBonus + 1.0) * rand() / (RAND_MAX + 1.0));
    int cur_wheel = 0;

    for(int i = 0; i < validConfigurations.size(); i++)
    {
      if (cur_wheel + (bonuses.at(i) * bonuses.at(i) * bonuses.at(i)) >= r_wheel)
      {
        winner = validConfigurations.at(i);
        break;
      }

      cur_wheel += bonuses.at(i) * bonuses.at(i) * bonuses.at(i);
    }

    //Increase all foot boni by one
    for(int i = 0; i < 6; i++) {
      bonus[i]++;
    }

    //Decrease the foot bonus that gets changed
    bonus[whichFoot(footConfiguration.state, winner)] = 0;

    signed char nextFootConf = winner;

    //Foot gets lifted up now
    if(footConfiguration.state > nextFootConf) 
    {
      cout << "Foot goes up: " << whichFoot(footConfiguration.state, nextFootConf) << endl;

      FootConfiguration nextConfiguration = footConfiguration.deepCopy();
      nextConfiguration.state = nextFootConf;

      std::vector<tf::Vector3> candidates;
      std::vector<int> candIdx;

      this->testCenterRegion(nextConfiguration, candidates, candIdx, centerList);

      assert(candidates.size() > 0);

      this->validCenterIdx = candIdx.back();
      this->validCenterPoint = candidates.back();
    }
    //Foot put down
    else {
      int wFoot = whichFoot(footConfiguration.state, nextFootConf);

      cout << "Foot goes down: " << wFoot << endl;

      assert("invariant: validCenterPoint MUST BE valid!" && (!this->outOfCenterRegion(this->validCenterPoint, footConfiguration)));

      this->validCenterPoint = this->endCenterPoint;
      this->validCenterIdx = this->endCenterIdx;

      tf::Vector3 startPoint = this->validCenterPoint + footConfiguration.get()[wFoot].initPosition;

      float r1 = (float) rand() / (float) RAND_MAX;
      float r2 = (float) rand() / (float) RAND_MAX;
      float r3 = (float) rand() / (float) RAND_MAX;
      float r4 = (float) rand() / (float) RAND_MAX;

      float drift = 0.05;

      tf::Vector3 rPos = tf::Vector3(drift * (r1-r2), drift * (r3-r4), 0.0);

      startPoint += rPos;

      //Find vector n moving to the goal position and add it to the default position
      tf::Vector3 goal = centerList.at(this->validCenterIdx);
      tf::Vector3 current = footConfiguration.get()[wFoot].initPosition;

      int p = this->validCenterIdx;
      if (p != 0) {
        p--;
        current = centerList.at(p);
      }

      tf::Vector3 direction = goal - current;
      
      direction.normalize();

      startPoint += (direction * 0.05);

      bool footGood = this->findValidFootPos(
        wFoot,
        startPoint,
        this->validCenterPoint,
        tf::Vector3(0.0, 0.008, 0.0),
        tf::Vector3(0.008, 0.0, 0.0),
        footConfiguration,
        10
      );

      if (!footGood)
      {
        cout << "createRandom2: foot not good\n";
        return false;
      }
    }

    footConfiguration.state = nextFootConf;

    assert("invariant: the new foot pos must be valid " && (!this->outOfCenterRegion(this->validCenterPoint, footConfiguration)));

    cPos = ControlRandomSampling::seg2Lin(this->validCenterIdx, this->validCenterPoint, centerList);

    this->calcCenterRange(centerList, footConfiguration);

    events.push_back(
      Event(
        footConfiguration.deepCopy(),
        this->startCenterPoint + tf::Vector3(0, 0, 0),
        this->endCenterPoint + tf::Vector3(0, 0, 0)
      )
    );
  } while((lin2Pos(cPos, start, centerList) - centerList.back()).length() > 0.01);

  std::vector<std::vector<tf::Vector3>> footPositions = {{}, {}, {}, {}, {}, {}};

  /*
  for(int i = 0; i < 100; i++) {
    footPositions[0].push_back(tf::Vector3(0,0,0.05));
    footPositions[1].push_back(tf::Vector3(0,-0.02,0));
    footPositions[2].push_back(tf::Vector3(0,-0.02,0));
    footPositions[3].push_back(tf::Vector3(0,0,0.05));
    footPositions[4].push_back(tf::Vector3(0,0,0.05));
    footPositions[5].push_back(tf::Vector3(0,-0.02,0));

    footPositions[0].push_back(tf::Vector3(0,0,0.05));
    footPositions[1].push_back(tf::Vector3(0,0.02,0));
    footPositions[2].push_back(tf::Vector3(0,0.02,0));
    footPositions[3].push_back(tf::Vector3(0,0,0.05));
    footPositions[4].push_back(tf::Vector3(0,0,0.05));
    footPositions[5].push_back(tf::Vector3(0,0.02,0));
  }*/

  std::vector<double> plotX,plotY;

  Event previous;

  bool first = true;
  int step = 1;
  for(Event e : events) {
    if(first) {
      first = false;
      previous = e;
      continue;
    } else {
      e.print();

      int changed = ControlRandomSampling::whichFoot(previous.footConfiguration.state, e.footConfiguration.state);
      std::string position = e.footConfiguration.state > previous.footConfiguration.state ? "▼": "▲";

      double fromX = previous.footConfiguration.get()[changed].position.x() + e.footConfiguration.get()[changed].delta.x();
      double fromY = previous.footConfiguration.get()[changed].position.y() + e.footConfiguration.get()[changed].delta.y();

      double toX = e.footConfiguration.get()[changed].position.x() + e.footConfiguration.get()[changed].delta.x();
      double toY = e.footConfiguration.get()[changed].position.y() + e.footConfiguration.get()[changed].delta.x();

      std::cout << "Foot " << changed << " went " << position;

      if(position == "▼") {
        std::cout.precision(2);
        std::cout << " from: (" << std::fixed << fromX << ", " << std::fixed << fromY << ") to (" << std::fixed << toX << ", " << std::fixed <<toY << ")" << std::endl;
      }
      else {
        std::cout << std::endl;
      }

      std::cout.precision(2);
      std::cout << "Center went from (" << std::fixed << e.start.x() << "," << std::fixed << e.start.y() << ") to (" << std::fixed << e.end.x() << "," << std::fixed << e.end.y() << ")" << std::endl;
    
      std::vector<double> centerX;
      std::vector<double> centerY;

      centerX.push_back(e.start.x());
      centerY.push_back(e.start.y());

      centerX.push_back(e.end.x());
      centerY.push_back(e.end.y());

      plt::plot(centerX, centerY);

      //only plot foot 0 and 1 for now
      //if(changed == 0 || changed==1) {
        std::vector<double> footX;
        std::vector<double> footY;

        footX.push_back(fromX);
        footY.push_back(fromY);

        footX.push_back(toX);
        footY.push_back(toY);
      
        plt::plot(footX, footY);
      //}

      for(int i = 0; i < 6; i++) {
        bool isFootDown = e.footConfiguration.state & FootConfiguration::FOOT_BIT[i];

        if(isFootDown) {
          footPositions[i].push_back(tf::Vector3(0,0,0.0));
        }
        else {
          footPositions[i].push_back(tf::Vector3(0,0,0.05));
        }
      }

      previous = e;
    }
    step++;
  }

  plt::legend();
  //plt::show();

  this->exportMovement(footPositions);

  return true;
}

bool ControlRandomSampling::findValidFootPos(
    int foot,
    const tf::Vector3& startPoint,
    const tf::Vector3& center,
    const tf::Vector3& dx,
    const tf::Vector3& dy,
    FootConfiguration& footConfiguration,
    int numCycles)
{

    //DEBUG
    std::vector<double> centerX;
    std::vector<double> centerY;

  footConfiguration.currentPosition[foot].position = startPoint - dx;
	footConfiguration.state |= FootConfiguration::FOOT_BIT[foot];

  centerX.push_back(footConfiguration.currentPosition[foot].position.x());
  centerY.push_back(footConfiguration.currentPosition[foot].position.y());
	if (!this->outOfCenterRegion(center, footConfiguration) && behindRule(footConfiguration))
	{
    //plt::plot(centerX, centerY); //DEBUG    
    //plt::show(); //DEBUG

		return true;
	}

	for (int r = 0; r < numCycles; r++)
	{
		footConfiguration.currentPosition[foot].position += dx;

    centerX.push_back(footConfiguration.currentPosition[foot].position.x());
  centerY.push_back(footConfiguration.currentPosition[foot].position.y());
		if (!this->outOfCenterRegion(center, footConfiguration) && behindRule(footConfiguration))
		{
      //plt::plot(centerX, centerY); //DEBUG    
    //plt::show(); //DEBUG
			return true;
		}
		for (int i = 0; i < (r * 2) - 1; i++)
		{
			footConfiguration.currentPosition[foot].position += dy;

      centerX.push_back(footConfiguration.currentPosition[foot].position.x());
  centerY.push_back(footConfiguration.currentPosition[foot].position.y());

			if (!this->outOfCenterRegion(center, footConfiguration) && behindRule(footConfiguration))
			{
        //plt::plot(centerX, centerY); //DEBUG    
    //plt::show(); //DEBUG
				return true;
			}
		}
		for (int i = 0; i < (r * 2); i++)
		{
			footConfiguration.currentPosition[foot].position -= dx;

      centerX.push_back(footConfiguration.currentPosition[foot].position.x());
  centerY.push_back(footConfiguration.currentPosition[foot].position.y());
			// test p if valid...
			if (!this->outOfCenterRegion(center, footConfiguration) && behindRule(footConfiguration))
			{
        //plt::plot(centerX, centerY); //DEBUG    
    //plt::show(); //DEBUG
				return true;
			}
		}
		for (int i = 0; i < (r * 2); i++)
		{
			footConfiguration.currentPosition[foot].position -= dy;

      centerX.push_back(footConfiguration.currentPosition[foot].position.x());
  centerY.push_back(footConfiguration.currentPosition[foot].position.y());
			// test p if valid...
			if (!this->outOfCenterRegion(center, footConfiguration) && behindRule(footConfiguration))
			{
        //plt::plot(centerX, centerY); //DEBUG    
    //plt::show(); //DEBUG
				return true;
			}
		}
		for (int i = 0; i < (r * 2); i++)
		{
			footConfiguration.currentPosition[foot].position += dx;

      centerX.push_back(footConfiguration.currentPosition[foot].position.x());
  centerY.push_back(footConfiguration.currentPosition[foot].position.y());
			// test p if valid...
			if (!this->outOfCenterRegion(center, footConfiguration) && behindRule(footConfiguration))
			{
        //plt::plot(centerX, centerY); //DEBUG    
    //plt::show(); //DEBUG
				return true;
			}
		}
	}

  //plt::plot(centerX, centerY); //DEBUG    
    //plt::show(); //DEBUG
	return false;
}

bool ControlRandomSampling::behindRule(FootConfiguration& conf) const
{
	float MIN_LEG_DIST = 0.02;
	
  if ((conf.state & FootConfiguration::FOOT_BIT[0]) && (conf.state & FootConfiguration::FOOT_BIT[2]) && (conf.get()[0].position.y() + conf.get()[0].delta.y() < conf.get()[2].position.y() + conf.get()[2].delta.y() + MIN_LEG_DIST))
	{
		return false;
	}
	if ((conf.state & FootConfiguration::FOOT_BIT[1]) && (conf.state & FootConfiguration::FOOT_BIT[3]) && (conf.get()[1].position.y() + conf.get()[1].delta.y() < conf.get()[3].position.y() + conf.get()[3].delta.y() + MIN_LEG_DIST))
	{
		return false;
	}
	if ((conf.state & FootConfiguration::FOOT_BIT[2]) && (conf.state & FootConfiguration::FOOT_BIT[4]) && (conf.get()[2].position.y() + conf.get()[2].delta.y() < conf.get()[4].position.y() + conf.get()[4].delta.y() + MIN_LEG_DIST))
	{
		return false;
	}
	if ((conf.state & FootConfiguration::FOOT_BIT[3]) && (conf.state & FootConfiguration::FOOT_BIT[5]) && (conf.get()[3].position.y() + conf.get()[3].delta.y() < conf.get()[5].position.y() + conf.get()[5].delta.y() + MIN_LEG_DIST))
	{
		return false;
	}

	return true;
}


void ControlRandomSampling::testCenterRegion(
  const FootConfiguration& footConfiguration,
  std::vector<tf::Vector3>& candidates,
  std::vector<int>& candIdx,
  const std::vector<tf::Vector3>& centerList
  )
{
	this->validCenterPoint = this->startCenterPoint;
	this->validCenterIdx = this->startCenterIdx;

	if (!this->outOfCenterRegion(this->validCenterPoint, footConfiguration))
	{
    candIdx.push_back(this->validCenterIdx);
    candidates.push_back(this->validCenterPoint);
	}
	while (this->validCenterIdx != this->endCenterIdx)
	{
		if (!this->outOfCenterRegion(this->validCenterPoint, footConfiguration))
		{
      candIdx.push_back(this->validCenterIdx);
      candidates.push_back(this->validCenterPoint);
		}

		this->validCenterPoint = centerList.at(this->validCenterIdx);
		this->validCenterIdx++;
	}
	if (!this->outOfCenterRegion(this->endCenterPoint, footConfiguration))
	{
    candIdx.push_back(this->endCenterIdx);
    candidates.push_back(this->endCenterPoint);
	}
}

std::vector<tf::Vector3> ControlRandomSampling::generateCenterPoints(tf::Vector3 start, tf::Vector3 end) const
{
  std::vector<tf::Vector3> points;

  tf::Vector3 path = end - start;
  tf::Vector3 mid = start + (path / 2);
  
  std::random_device rd;
  std::mt19937 generator(rd());
  std::geometric_distribution<int> geometricDistribution(0.5);
  std::uniform_real_distribution<double> randomDistribution(0.0, 1.0);
  
  int segments = 0;//geometricDistribution(generator);
  
  for (int i = 0; i < segments; i++)
  {
    points.push_back(
      mid +
      tf::Vector3(
        randomDistribution(generator) * path.length() - randomDistribution(generator) * path.length(),
        randomDistribution(generator) * path.length() - randomDistribution(generator) * path.length(),
        0
      )
    );
  }
  
  points.push_back(end);
  
  return points;
}

void ControlRandomSampling::calcCenterRange(
  std::vector<tf::Vector3> centerList,
  FootConfiguration footConfiguration
) {
  assert (this->validCenterIdx != centerList.size());
  assert(!ControlRandomSampling::outOfCenterRegion(this->validCenterPoint, footConfiguration));

  this->startCenterIdx = this->validCenterIdx;

	while(this->startCenterIdx != 0)
	{
		this->startCenterIdx--;
		if (this->outOfCenterRegion(centerList.at(this->startCenterIdx), footConfiguration))
		{
			this->startCenterPoint = centerList.at(this->startCenterIdx);
			this->startCenterIdx++;

			break;
		}
	}

  if (this->startCenterIdx == 0)
	{
		this->startCenterPoint = tf::Vector3(0.0, 0.0, 0.0);
	}

  tf::Vector3 startInnerPoint;
  tf::Vector3 startOuterPoint;

  if (this->startCenterIdx == this->validCenterIdx)
	{
		startInnerPoint = this->validCenterPoint;
	} else {
		startInnerPoint = centerList.at(this->startCenterIdx);
	}

  assert (!this->outOfCenterRegion(startInnerPoint, footConfiguration));

  startOuterPoint = this->startCenterPoint;

  for (int i = 0; i < 16; i++) {
    tf::Vector3 midPoint = (startInnerPoint + startOuterPoint) * 0.5;

    if (ControlRandomSampling::outOfCenterRegion(midPoint, footConfiguration))
      {
        startOuterPoint = midPoint;
      } else {
        startInnerPoint = midPoint;
      }
  }

  this->startCenterPoint = startInnerPoint;

  assert (!this->outOfCenterRegion(this->startCenterPoint, footConfiguration));

	// Now for the other index direction
  this->endCenterIdx = this->validCenterIdx;
  
	while (this->endCenterIdx != centerList.size())
	{
		if (this->outOfCenterRegion(centerList.at(this->endCenterIdx), footConfiguration))
		{
			this->endCenterPoint = centerList.at(this->endCenterIdx);
			break;
		}
		this->endCenterIdx++;
	}

	if (this->endCenterIdx == centerList.size())
	{
		this->endCenterIdx--;
		this->endCenterPoint = centerList.at(this->endCenterIdx);
	}

	tf::Vector3 endInnerPoint;
	tf::Vector3 endOuterPoint;

	endOuterPoint = this->endCenterPoint;

	if (this->endCenterIdx == this->validCenterIdx)
	{
		endInnerPoint = this->validCenterPoint;
	} else {
		endInnerPoint = centerList.at(this->endCenterIdx - 1);
	}

	// step-by-step approximation
	for (int i = 0; i < 16; i++)
	{
		tf::Vector3 midPoint = (endInnerPoint + endOuterPoint) * 0.5;

		if (this->outOfCenterRegion(midPoint, footConfiguration))
		{
			endOuterPoint = midPoint;
		} else {
			endInnerPoint = midPoint;
		}
	}
	this->endCenterPoint = endInnerPoint;
}

int ControlRandomSampling::whichFoot(signed char current, signed char next)
{
	int foot = -1;
	for (int i = 0; i < 6; i++) {
		if ((current ^ next) == FootConfiguration::FOOT_BIT[i])
		{
			foot = i;
		}
	}
  
	return foot;
}

bool ControlRandomSampling::outOfCenterRegion(tf::Vector3 center, FootConfiguration footConfiguration) const
{
  std::vector<tf::Vector3> points;

  for(size_t i = 0; i < footConfiguration.currentPosition.size(); i++)
  {
    if(footConfiguration.state & FootConfiguration::FOOT_BIT[i]) {
      points.push_back(footConfiguration.currentPosition[i].position + footConfiguration.currentPosition[i].delta - center);
    }
  }

  assert(points.size() >= 3);

  // Test 1: Stability Margin
  std::vector<tf::Vector3> hullPoints = convexHull(points);

  assert(hullPoints.size() >= 3);

  // Calculate the smallest orthogonal distance
  float smallestDistance = std::numeric_limits<float>::max();
  
  for (size_t i = 0; i < hullPoints.size(); i++)
  {
    int position = i > 0 ? i - 1 : hullPoints.size() - 1;
    
    tf::Vector3 a = hullPoints.at(position);
    tf::Vector3 b = hullPoints.at(i);
    
    tf::Vector3 r = a - b;
    tf::Vector3 rOrtho = tf::Vector3(-r.y(), r.x(), 0);
    
    float distance = (b.x() * (b.y() - a.y()) + (b.y() * (a.x() - b.x()))) / rOrtho.length();

    if (distance < smallestDistance)
    {
			smallestDistance = distance;
		}
	}

  if(smallestDistance < 0.051)
  {
    return true;
  }
  
  // Center Region
  double rangeMarging = 0.020;

	for(int i = 0; i < footConfiguration.get().size(); i++) 
  {
    FootConfiguration::Foot foot = footConfiguration.get()[i];

    tf::Vector3 relativeFootPosition = tf::Vector3(
      abs(foot.position.x() - center.x()),
      abs(foot.position.y() - center.y()),
      foot.position.z() - center.z()
    );

    if(footConfiguration.state & FootConfiguration::FOOT_BIT[i]) {
      if(
        !akrobat.anglesValid(i, akrobat.inverseKinematics(relativeFootPosition + tf::Vector3(rangeMarging, rangeMarging, 0))) || 
        !akrobat.anglesValid(i, akrobat.inverseKinematics(relativeFootPosition + tf::Vector3(-rangeMarging, rangeMarging, 0))) || 
        !akrobat.anglesValid(i, akrobat.inverseKinematics(relativeFootPosition + tf::Vector3(rangeMarging, -rangeMarging, 0))) || 
        !akrobat.anglesValid(i, akrobat.inverseKinematics(relativeFootPosition + tf::Vector3(-rangeMarging, -rangeMarging, 0))) || 
        !akrobat.anglesValid(i, akrobat.inverseKinematics(relativeFootPosition))
      ) {
        return true;
      }
    }
  }

  return false;
}

std::vector<tf::Vector3> ControlRandomSampling::convexHull(const std::vector<tf::Vector3>& points) const
{
  std::vector<tf::Vector3> hullPoints;
  
  // With a size <= 2 you already have the hull
  if (points.size() <= 2) {
    for(const tf::Vector3& point : points) {
      hullPoints.push_back(point);
    }
	
		return hullPoints;
	}
  
  // Find and put the leftmost point in the hull
  hullPoints.push_back(points[0]);
  
  for(const tf::Vector3& point : points) {
    if(point.x() < hullPoints[0].x() || (point.x() == hullPoints[0].x() && (point.y() < hullPoints[0].y()))) {
      hullPoints[0] = point;
    }
  }
  
  // Find the other points until reaching the first point
  do {
  	tf::Vector3 bestPoint = points.at(0);
    
    for(const tf::Vector3& point : points)
  	{
      tf::Vector3 lastPoint = hullPoints.at(hullPoints.size() - 1);
      tf::Vector3 testPoint = point;
      
      float orientation = (((bestPoint.x() - lastPoint.x()) * (testPoint.y() - lastPoint.y())) - ((bestPoint.y() - lastPoint.y()) * (testPoint.x() - lastPoint.x())));
      
  		if ((orientation < .0) || ((orientation == .0) && ((bestPoint - lastPoint).length() < (point - lastPoint).length())))
  		{
  			bestPoint = point;
  		}
  	}
    
  	hullPoints.push_back(bestPoint);
  } while (hullPoints.at(0) != hullPoints.at(hullPoints.size() - 1));
  
  // Remove last point of hull
  hullPoints.pop_back();
  
  return hullPoints;
}

tf::Vector3 ControlRandomSampling::lin2Pos(float lin, tf::Vector3 center, std::vector<tf::Vector3> centerList) const
{
	tf::Vector3 curpos = center;
	float len = 0;

	for(tf::Vector3 centerElement : centerList)
	{
    tf::Vector3 delta = centerElement - curpos;
		const float delta_len = delta.length();

		if (len + delta_len > lin)
		{
			return curpos + (delta * (lin - len) / delta_len);
		}

		len += delta_len;
		curpos = centerElement;
	}
	
	return centerList.back();
}

float ControlRandomSampling::seg2Lin(int& seg, tf::Vector3& point, std::vector<tf::Vector3>& centerList) const
{
  tf::Vector3 currentPosition = tf::Vector3(0.0, 0.0, 0.0);

  float len = 0;

  for(int i = 0; i < centerList.size(); i++)
  {
    if(i == seg) {
      return len + (point - currentPosition).length();
    }
    len += (centerList[i] - currentPosition).length();
    currentPosition = centerList[i];
  }

  return -1.0;
}