#ifndef CONTROLRANDOMSAMPLING_H
#define CONTROLRANDOMSAMPLING_H

#include <string>
#include <vector>

#include <ros/ros.h>
#include <akrobat/Akrobat.h>
#include "akrobat/FootConfiguration.h"

#include <tf/tf.h>
#include <tf/transform_datatypes.h>



class ControlRandomSampling
{
public:
  ControlRandomSampling(Akrobat& akrobat);
  
  std::vector<std::vector<tf::Vector3>> readMovement() const;
  void exportMovement(std::vector<std::vector<tf::Vector3>>& footPoints) const;
  
  void tripodGait() const;
  void randomSampling();
  bool generateRandomSampling();
  
  bool outOfCenterRegion(tf::Vector3 center, FootConfiguration footConfiguration) const;
  
  void calcCenterRange(
    std::vector<tf::Vector3> centerList,
    FootConfiguration footConfiguration
  );

  void testCenterRegion(const FootConfiguration& footConfiguration, std::vector<tf::Vector3>& candidates, std::vector<int>& candIdx, const std::vector<tf::Vector3>& centerList);

  static int whichFoot(signed char current, signed char next);
  std::vector<tf::Vector3> generateCenterPoints(tf::Vector3 start, tf::Vector3 end) const;
  float getStabilityMargin(const std::vector<tf::Vector3>& points) const;
  std::vector<tf::Vector3> convexHull(const std::vector<tf::Vector3>& positions) const;
  
  bool findValidFootPos(
    int foot,
    const tf::Vector3& startPoint,
    const tf::Vector3& center,
    const tf::Vector3& dx,
    const tf::Vector3& dy,
    FootConfiguration& footConfiguration,
    int numCycles
  );

  bool behindRule(FootConfiguration& conf) const;
  tf::Vector3 lin2Pos(float lin, tf::Vector3 center, std::vector<tf::Vector3> centerList) const;
  float seg2Lin(int& seg, tf::Vector3& point, std::vector<tf::Vector3>& centerList) const;

  struct Event {
    Event() {}
    Event(const FootConfiguration& footConfiguration, const tf::Vector3& start, const tf::Vector3& end)
        : footConfiguration(footConfiguration), start(start), end(end)
    {}

    FootConfiguration footConfiguration;
    tf::Vector3 start;
    tf::Vector3 end;

    void print() const {
      for(int i = 0; i < 6; i++) {
        std::cout
          << "Foot:" << i << "\t"
          << (footConfiguration.get()[i].position.x() - footConfiguration.get()[i].initPosition.x()) << "\t"
          << (footConfiguration.get()[i].position.y() - footConfiguration.get()[i].initPosition.y()) << "\t"
          << (footConfiguration.get()[i].position.z() - footConfiguration.get()[i].initPosition.z()) << "\t"
          << (footConfiguration.state & FootConfiguration::FOOT_BIT[i] ? " down": " up")
          << std::endl;
      }

      std::cout << "Start\t" << start.x() << "\t" << start.y() << "\t" << start.z() << std::endl;
      std::cout << "End\t" << end.x() << "\t" << end.y() << "\t" << end.z() << std::endl;
      std::cout << std::endl;
    }
  };

private:
  Akrobat& akrobat;

  int validCenterIdx;
  int startCenterIdx;
  int endCenterIdx;

  tf::Vector3 validCenterPoint;
  tf::Vector3 startCenterPoint;
  tf::Vector3 endCenterPoint;
};

#endif