# OctoManager #

This class offers a simple Octomap wrapper, adding some useful features such as inflating the occupied nodes given a certain radius or checking if obstacles exist between two existing nodes.

Moreover, this node has two modes to work with:

* **Sensor mode**: It sets a callback function to create and update an Octomap using a pointcloud from an user-defined topic. Note that to work properly, a correct tf between the sensor and the map must be published. In this mode inflation is unavailable.
* **Map mode**: When a .pcd path is defined in the config file, an Octomap is created (and published for visualization purposes) from this pointcloud. After that, the given map will be inflated given a certain user-defined radius, in meters. The ground can also be removed before inflating if needed.

This code is based on [octomap_mapping](http://wiki.ros.org/octomap_mapping) so it's strongly recommended to check if out as well.

Of course, this code can be easily integrated into any other node, so it does not have to run as an independent node, just by copying both **.cpp** and **.h** files into yours.

## Dependencies
* ROS Noetic [(Ubuntu 20.04)](http://wiki.ros.org/noetic/Installation/Ubuntu)

* [Octomap](http://wiki.ros.org/octomap)
 
* [PCL](http://wiki.ros.org/pcl_ros)

## Topics

### Published topics
|name|type|
|----|----|
| /occupied_octomap_cells | [visualization_msgs/MarkerArray](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html) |
| /occupied_octomap_cells/inflated_nodes | [visualization_msgs/MarkerArray](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html) |

## Services
|name|type|
|----|----|
| /check_obstacles_between_wp| [octomanager/CheckObstacles](https://bitbucket.org/fadacatec-ondemand/octomanager/src/master/) |

## Installation

### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using:

```console
$ cd catkin_workspace
$ catkin_make -DCMAKE_BUILD_TYPE=Release
```

### Launch files

* **octomanager.launch:** Starts the octomanager node as well as an rviz window for visualization, and reads the parameters value from the **params.yaml** file.

```console
$ roslaunch octomanager octomanager.launch
```

## Params configuration:

```
lidar_frame_id: "os_lidar" # Name of the sensor frame
global_frame_id: "map" # Name of the map origin frame

lidar_topic: "/os_node/points" # LiDAR sensor topic
octomap_vis_topic: "/occupied_octomap_cells" # Visual octomap topic
pcd_path: "" # Leave empty if you're working with a sensor pointcloud

# --- Octomap parameters
resolution: 0.1 # [m]
prob_hit: 0.7 #
prob_miss: 0.4 #
clamping_thresh_min: 0.12 #
clamping_thresh_max: 0.97 #
# ---

inflation_radius: 0 # [m] Set 0 to ignore inflation
ground_removal_dist: 0 # [m] Set 0 to ignore ground removal

# --- Does not work when a pcd_path is available (Map mode)
min_sensor_range: # [m]
max_sensor_range: # [m]
# ---
```