#ifndef BEST_FRONTIER_H_
#define BEST_FRONTIER_H_

#include <ros/ros.h>
#include <octomap_msgs/Octomap.h>
#include <octomanager/params.h>

#include <octomap_ros/conversions.h>
#include <nav_msgs/OccupancyGrid.h>

#include <octomap/octomap.h>
#include "yaml-cpp/yaml.h"

using namespace std;
using namespace octomap;
using octomap_msgs::Octomap;


namespace best_frontier
{
  class BestFrontier
  {
    public:
      BestFrontier();
      // bool configureFromFile(string config_filename);
      point3d bestFrontierInfGain(
        octomap::OcTree* octree, point3d currentPosition, KeySet& Cells);
  
    protected:
      double calculateDistance(const point3d p1, const point3d p2)
      {		
        double distance = sqrt(
        pow(p2.x() - p1.x(), 2) +
        pow(p2.y() - p1.y(), 2) +
        pow(p2.z() - p1.z(), 2));
        return distance;
      }
      double calcMIBox(const octomap::OcTree *octree, const point3d &sensorOrigin); 
      
      // string m_configFilename;bbx_minX
      // double m_resolution, m_kGain, m_lambda, m_boxInfGainSize;
      // ofstream m_logfile; 
      catec::Parameters _params;
    };
}
#endif