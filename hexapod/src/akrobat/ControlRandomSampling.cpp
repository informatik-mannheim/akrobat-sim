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
//#include "matplotlib-cpp/matplotlibcpp.h"
//namespace plt = matplotlibcpp;

//end test includes

using namespace std;
using namespace ros;

ControlRandomSampling::ControlRandomSampling(Akrobat &akrobat) : akrobat(akrobat),
                                                                 validCenterIdx(0),
                                                                 startCenterIdx(0),
                                                                 endCenterIdx(0),
                                                                 validCenterPoint(tf::Vector3(0, 0, 0)),
                                                                 startCenterPoint(tf::Vector3(0, 0, 0)),
                                                                 endCenterPoint(tf::Vector3(0, 0, 0))
{
}

void ControlRandomSampling::forwardMovement() const
{
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
  traData.ampY[0] = traData.initAmpY;
  traData.ampZ[0] = traData.initAmpZ;

  traData.ampX[1] = traData.initAmpX * 0;
  traData.ampY[1] = traData.initAmpY;
  traData.ampZ[1] = traData.initAmpZ;

  traData.ampX[2] = traData.initAmpX * 0;
  traData.ampY[2] = traData.initAmpY;
  traData.ampZ[2] = traData.initAmpZ;

  traData.ampX[3] = traData.initAmpX * 0;
  traData.ampY[3] = traData.initAmpY;
  traData.ampZ[3] = traData.initAmpZ;

  traData.ampX[4] = traData.initAmpX * 0;
  traData.ampY[4] = traData.initAmpY;
  traData.ampZ[4] = traData.initAmpZ;

  traData.ampX[5] = traData.initAmpX * 0;
  traData.ampY[5] = traData.initAmpY;
  traData.ampZ[5] = traData.initAmpZ;

  std::vector<std::vector<tf::Vector3>> footPoints = {{}, {}, {}, {}, {}, {}};

  for (int step = 0; step < 100; step++)
  {
    for (int legNum = 0; legNum < 6; legNum++)
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

void ControlRandomSampling::stopMovement() const
{
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
  traData.ampY[0] = traData.initAmpY;
  traData.ampZ[0] = traData.initAmpZ;

  traData.ampX[1] = traData.initAmpX * 0;
  traData.ampY[1] = traData.initAmpY;
  traData.ampZ[1] = traData.initAmpZ;

  traData.ampX[2] = traData.initAmpX * 0;
  traData.ampY[2] = traData.initAmpY;
  traData.ampZ[2] = traData.initAmpZ;

  traData.ampX[3] = traData.initAmpX * 0;
  traData.ampY[3] = traData.initAmpY;
  traData.ampZ[3] = traData.initAmpZ;

  traData.ampX[4] = traData.initAmpX * 0;
  traData.ampY[4] = traData.initAmpY;
  traData.ampZ[4] = traData.initAmpZ;

  traData.ampX[5] = traData.initAmpX * 0;
  traData.ampY[5] = traData.initAmpY;
  traData.ampZ[5] = traData.initAmpZ;

  std::vector<std::vector<tf::Vector3>> footPoints = {{}, {}, {}, {}, {}, {}};

  for (int step = 0; step < 100; step++)
  {
    for (int legNum = 0; legNum < 6; legNum++)
    {
      footPoints.at(legNum).push_back(tf::Vector3(traData.ampX[legNum], traData.ampY[legNum], traData.ampZ[legNum]));
    }

    traData.tick++;
    if (traData.tick > 15 - 1)
    {
      traData.tick = 0;
    }
  }

  ControlRandomSampling::exportMovement(footPoints);
}

void ControlRandomSampling::backwardMovement() const
{
  Trajectory traData = Trajectory();

  traData.caseStep[0] = 2;
  traData.caseStep[1] = 1;
  traData.caseStep[2] = 1;
  traData.caseStep[3] = 2;
  traData.caseStep[4] = 2;
  traData.caseStep[5] = 1;

  traData.initAmpX = -0.040;
  traData.initAmpY = -0.040;
  traData.initAmpZ = -0.040;
  traData.tick = 0;

  traData.ampX[0] = traData.initAmpX * 0;
  traData.ampY[0] = traData.initAmpY;
  traData.ampZ[0] = traData.initAmpZ;

  traData.ampX[1] = traData.initAmpX * 0;
  traData.ampY[1] = traData.initAmpY;
  traData.ampZ[1] = traData.initAmpZ;

  traData.ampX[2] = traData.initAmpX * 0;
  traData.ampY[2] = traData.initAmpY;
  traData.ampZ[2] = traData.initAmpZ;

  traData.ampX[3] = traData.initAmpX * 0;
  traData.ampY[3] = traData.initAmpY;
  traData.ampZ[3] = traData.initAmpZ;

  traData.ampX[4] = traData.initAmpX * 0;
  traData.ampY[4] = traData.initAmpY;
  traData.ampZ[4] = traData.initAmpZ;

  traData.ampX[5] = traData.initAmpX * 0;
  traData.ampY[5] = traData.initAmpY;
  traData.ampZ[5] = traData.initAmpZ;

  std::vector<std::vector<tf::Vector3>> footPoints = {{}, {}, {}, {}, {}, {}};

  for (int step = 0; step < 100; step++)
  {
    for (int legNum = 0; legNum < 6; legNum++)
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
        x = traData.ampX[legNum] + 0.002 * traData.ampX[legNum];
        y = traData.ampY[legNum] + 0.002 * traData.ampY[legNum];
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
void ControlRandomSampling::leftTurn() const
{
  Trajectory traData = Trajectory();

  traData.caseStep[0] = 1;
  traData.caseStep[1] = 2;
  traData.caseStep[2] = 2;
  traData.caseStep[3] = 1;
  traData.caseStep[4] = 2;
  traData.caseStep[5] = 1;

  traData.initAmpX = 0.04;
  traData.initAmpY = 0.04;
  traData.initAmpZ = 0.04;
  traData.tick = 0;

  traData.ampX[0] = traData.initAmpX;
  traData.ampY[0] = traData.initAmpY;
  traData.ampZ[0] = traData.initAmpZ;

  traData.ampX[1] = traData.initAmpX;
  traData.ampY[1] = traData.initAmpY;
  traData.ampZ[1] = traData.initAmpZ;

  traData.ampX[2] = traData.initAmpX;
  traData.ampY[2] = traData.initAmpY;
  traData.ampZ[2] = traData.initAmpZ;

  traData.ampX[3] = traData.initAmpX;
  traData.ampY[3] = traData.initAmpY;
  traData.ampZ[3] = traData.initAmpZ;

  traData.ampX[4] = traData.initAmpX;
  traData.ampY[4] = traData.initAmpY;
  traData.ampZ[4] = traData.initAmpZ;

  traData.ampX[5] = traData.initAmpX;
  traData.ampY[5] = traData.initAmpY;
  traData.ampZ[5] = traData.initAmpZ;

  std::vector<std::vector<tf::Vector3>> footPoints = {{}, {}, {}, {}, {}, {}};

  for (int step = 0; step < 100; step++)
  {
    for (int legNum = 0; legNum < 6; legNum++)
    {
      float x;
      float y;
      float z;

      switch (traData.caseStep[legNum])
      {

      case 1:
        if (legNum == 0 || legNum == 2 || legNum == 4)
        {
          x = -traData.ampX[legNum];
          y = -traData.ampY[legNum];
        }
        else if (legNum == 1 || legNum == 3 || legNum == 5)
        {
          x = traData.ampX[legNum];
          y = traData.ampY[legNum];
        }

        z = abs(traData.ampZ[legNum]);

        traData.caseStep[legNum] = 2;
        break;

      case 2:
        if (legNum == 0 || legNum == 2 || legNum == 4)
        {
          x = traData.ampX[legNum] - 0.004 * traData.ampX[legNum];
          y = traData.ampY[legNum] - 0, 004 * traData.ampY[legNum];
        }
        else if (legNum == 1 || legNum == 3 || legNum == 5)
        {
          x = traData.ampX[legNum] + 0.004 * traData.ampX[legNum];
          y = traData.ampY[legNum] + 0, 004 * traData.ampY[legNum];
        }
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
void ControlRandomSampling::rightTurn() const
{
  Trajectory traData = Trajectory();

  traData.caseStep[0] = 1;
  traData.caseStep[1] = 2;
  traData.caseStep[2] = 2;
  traData.caseStep[3] = 1;
  traData.caseStep[4] = 2;
  traData.caseStep[5] = 1;

  traData.initAmpX = 0.04;
  traData.initAmpY = 0.04;
  traData.initAmpZ = 0.04;
  traData.tick = 0;

  traData.ampX[0] = traData.initAmpX;
  traData.ampY[0] = traData.initAmpY;
  traData.ampZ[0] = traData.initAmpZ;

  traData.ampX[1] = traData.initAmpX;
  traData.ampY[1] = traData.initAmpY;
  traData.ampZ[1] = traData.initAmpZ;

  traData.ampX[2] = traData.initAmpX;
  traData.ampY[2] = traData.initAmpY;
  traData.ampZ[2] = traData.initAmpZ;

  traData.ampX[3] = traData.initAmpX;
  traData.ampY[3] = traData.initAmpY;
  traData.ampZ[3] = traData.initAmpZ;

  traData.ampX[4] = traData.initAmpX;
  traData.ampY[4] = traData.initAmpY;
  traData.ampZ[4] = traData.initAmpZ;

  traData.ampX[5] = traData.initAmpX;
  traData.ampY[5] = traData.initAmpY;
  traData.ampZ[5] = traData.initAmpZ;

  std::vector<std::vector<tf::Vector3>> footPoints = {{}, {}, {}, {}, {}, {}};

  for (int step = 0; step < 100; step++)
  {
    for (int legNum = 0; legNum < 6; legNum++)
    {
      float x;
      float y;
      float z;

      switch (traData.caseStep[legNum])
      {

      case 1:
        if (legNum == 0 || legNum == 2 || legNum == 4)
        {
          x = traData.ampX[legNum];
          y = traData.ampY[legNum];
        }
        else if (legNum == 1 || legNum == 3 || legNum == 5)
        {
          x = -traData.ampX[legNum];
          y = -traData.ampY[legNum];
        }

        z = abs(traData.ampZ[legNum]);

        traData.caseStep[legNum] = 2;
        break;

      case 2:
        if (legNum == 0 || legNum == 2 || legNum == 4)
        {
          x = traData.ampX[legNum] + 0.004 * traData.ampX[legNum];
          y = traData.ampY[legNum] + 0, 004 * traData.ampY[legNum];
        }
        else if (legNum == 1 || legNum == 3 || legNum == 5)
        {
          x = traData.ampX[legNum] - 0.004 * traData.ampX[legNum];
          y = traData.ampY[legNum] - 0, 004 * traData.ampY[legNum];
        }
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
void ControlRandomSampling::tripodGait() const
{
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
  traData.ampY[0] = traData.initAmpY;
  traData.ampZ[0] = traData.initAmpZ;

  traData.ampX[1] = traData.initAmpX * 0;
  traData.ampY[1] = traData.initAmpY;
  traData.ampZ[1] = traData.initAmpZ;

  traData.ampX[2] = traData.initAmpX * 0;
  traData.ampY[2] = traData.initAmpY;
  traData.ampZ[2] = traData.initAmpZ;

  traData.ampX[3] = traData.initAmpX * 0;
  traData.ampY[3] = traData.initAmpY;
  traData.ampZ[3] = traData.initAmpZ;

  traData.ampX[4] = traData.initAmpX * 0;
  traData.ampY[4] = traData.initAmpY;
  traData.ampZ[4] = traData.initAmpZ;

  traData.ampX[5] = traData.initAmpX * 0;
  traData.ampY[5] = traData.initAmpY;
  traData.ampZ[5] = traData.initAmpZ;

  std::vector<std::vector<tf::Vector3>> footPoints = {{}, {}, {}, {}, {}, {}};

  for (int step = 0; step < 100; step++)
  {
    for (int legNum = 0; legNum < 6; legNum++)
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

std::vector<std::vector<tf::Vector3>> ControlRandomSampling::readMovement() const
{
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
          std::stof(step.child_value("z")));

      footPoints.at(footIndex).push_back(point);
    }
  }

  return footPoints;
}

void ControlRandomSampling::exportMovement(std::vector<std::vector<tf::Vector3>> &footPoints) const
{
  // Writing start and end points to a movement file
  ofstream movementFile;
  movementFile.open("movements.xml");

  movementFile << "<?xml version=\"1.0\"?>" << endl;
  movementFile << "<movement>" << endl;

  for (size_t i = 0; i < footPoints.size(); i++)
  {
    movementFile << "\t<foot number=\"" << i << "\">" << endl;
    for (size_t j = 0; j < footPoints.at(i).size(); j++)
    {
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
