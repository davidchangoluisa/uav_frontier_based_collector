#include "ros/ros.h"
#include <mrs_msgs/TrajectoryReference.h>
#include <iostream>

int main (int argc, char **argv)

{
    ros::init (argc, argv, "trajectory_horus");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise <mrs_msgs::TrajectoryReference>("/uav1/control_manager/trajectory_reference",10);

    ros::spinOnce();
    ros::Duration(1.0).sleep();

    mrs_msgs::TrajectoryReference traj;
    mrs_msgs::Reference pt;
    traj.fly_now=true;
    traj.dt=5;
    // traj.loop=5;

    pt.position.x=1.0;
    pt.position.y=1.0;
    pt.position.z=2.0;
    traj.points.push_back( pt );

    pt.position.x=2.0;
    pt.position.y=1.0;
    traj.points.push_back( pt );

    pt.position.x=2.0;
    pt.position.y=2.0;
    traj.points.push_back( pt );

    pt.position.x=1.0;
    pt.position.y=2.0;
    traj.points.push_back( pt );

    pt.position.x=1.0;
    pt.position.y=1.0;
    traj.points.push_back( pt );

    pub.publish(traj);

    ros::spin();


    return 0;
}