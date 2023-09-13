#include <octomanager/params.h>
#include <ros/ros.h>

namespace catec {
Parameters::Parameters()
{
    if (!ros::param::get("~global_frame_id", global_frame_id))
        exitWithParameterError("global_frame_id");

    if (!ros::param::get("~lidar_frame_id", lidar_frame_id))
        exitWithParameterError("lidar_frame_id");

    if (!ros::param::get("~lidar_topic", lidar_topic))
        exitWithParameterError("lidar_topic");
    
    if (!ros::param::get("~odom_topic", odom_topic))
        exitWithParameterError("odom_topic");

    if (!ros::param::get("~octomap_vis_topic", octomap_vis_topic))
        exitWithParameterError("octomap_vis_topic");

    if (!ros::param::get("~pcd_path", pcd_path))
        exitWithParameterError("pcd_path");

    if (!ros::param::get("~resolution", resolution))
        exitWithParameterError("resolution");

    if (!ros::param::get("~min_sensor_range", min_sensor_range))
        exitWithParameterError("min_sensor_range");

    if (!ros::param::get("~max_sensor_range", max_sensor_range))
        exitWithParameterError("max_sensor_range");

    if (!ros::param::get("~prob_hit", prob_hit))
        exitWithParameterError("prob_hit");

    if (!ros::param::get("~prob_miss", prob_miss))
        exitWithParameterError("prob_miss");

    if (!ros::param::get("~clamping_thresh_min", clamping_thresh_min))
        exitWithParameterError("clamping_thresh_min");

    if (!ros::param::get("~clamping_thresh_max", clamping_thresh_max))
        exitWithParameterError("clamping_thresh_max");

    if (!ros::param::get("~inflation_radius", inflation_radius))
        exitWithParameterError("inflation_radius");

    if (!ros::param::get("~ground_removal_dist", ground_removal_dist))
        exitWithParameterError("ground_removal_dist");
    
    if (!ros::param::get("~bbx_minX", bbx_minX))
        exitWithParameterError("bbx_minX");
    if (!ros::param::get("~bbx_maxX", bbx_maxX))
        exitWithParameterError("bbx_maxX");
    if (!ros::param::get("~bbx_minY", bbx_minY))
        exitWithParameterError("bbx_minY");
    if (!ros::param::get("~bbx_maxY", bbx_maxY))
        exitWithParameterError("bbx_maxY");
    if (!ros::param::get("~bbx_minZ", bbx_minZ))
        exitWithParameterError("bbx_minZ");
    if (!ros::param::get("~bbx_maxZ", bbx_maxZ))
        exitWithParameterError("bbx_maxZ");
    
    if (!ros::param::get("~kernel_bandwidth", kernel_bandwidth))
    exitWithParameterError("kernel_bandwidth");

    if (!ros::param::get("~kGain", kGain))
    exitWithParameterError("kGain");

    if (!ros::param::get("~lambda", lambda))
    exitWithParameterError("lambda");

    if (!ros::param::get("~boxInfGainSize", boxInfGainSize))
    exitWithParameterError("boxInfGainSize");
    
    // if (!ros::param::get("~ground_removal_dist", ground_removal_dist))
    //     exitWithParameterError("ground_removal_dist");

    // if (!ros::param::get("~ground_removal_dist", ground_removal_dist))
    //     exitWithParameterError("ground_removal_dist");

    // if (!ros::param::get("~ground_removal_dist", ground_removal_dist))
    //     exitWithParameterError("ground_removal_dist");

    
}

void Parameters::exitWithParameterError(const char* str)
{
    ROS_ERROR("[%s] `%s` parameter not set!", ros::this_node::getName().data(), str);
    exit(EXIT_FAILURE);
}
} // namespace catec