#pragma once

#include <string>

namespace catec {
class Parameters {
public:
    Parameters();

    std::string lidar_frame_id;
    std::string global_frame_id;

    std::string lidar_topic;
    std::string octomap_vis_topic;
    std::string pcd_path;
    std::string odom_topic;

    double min_sensor_range, max_sensor_range;

    double resolution, inflation_radius;
    double prob_hit, prob_miss;
    double clamping_thresh_min, clamping_thresh_max;
    double ground_removal_dist;

    //Exploration parameters
    double bbx_minX;
    double bbx_maxX;
    double bbx_minY; 
    double bbx_maxY;
    double bbx_minZ;
    double bbx_maxZ;

    unsigned explorationDepth;
    double kernel_bandwidth;

    double kGain;
    double lambda;
    double boxInfGainSize;


private:
    void exitWithParameterError(const char* str);
};
} // namespace catec