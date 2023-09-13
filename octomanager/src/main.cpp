#include <octomanager/octomanager.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "octomanager_node");

    if (!ros::master::check()) {
        ROS_ERROR("No ROS master available.");
        return -1;
    }

    ros::start();

    catec::Octomanager node;
    node.run();

    return 0;
}