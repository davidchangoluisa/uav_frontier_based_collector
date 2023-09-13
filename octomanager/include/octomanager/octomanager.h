#pragma once

#include <octomanager/params.h>
#include <octomanager/BestFrontier.h>

#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>

#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/ColorRGBA.h>
#include <std_srvs/SetBool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Bool.h>

#include <octomanager/CheckObstacles.h>
#include <octomanager/ClusteringAlgorithm.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>
#include <octomap/OcTree.h>
#include <octomap/OcTreeLUT.h>
#include <pcl/common/common.h>




#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/io/pcd_io.h>
#include <pcl/common/distances.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/angles.h>

#include <algorithm>
#include <vector>


using namespace std;
using namespace octomap;
using octomap_msgs::Octomap;

namespace catec {

    enum ExplorationState{
        OFF,
        CHECKFORFRONTIERS,
        ON,
        POINTREACHED
    };

    inline const char* ToString(ExplorationState v)
    {
        switch(v)
        {
            case OFF: return "OFF - Waiting for servis call";
            case CHECKFORFRONTIERS: return "CHECKFORFRONTIERS - Waiting for frontiers";
            case ON:                return "ON - calculate best frontier";
            case POINTREACHED:      return "POINTREACHED - Current goal point is reached";
            default:                return "[Unknown ExplorationState]";
        }
    }

class Octomanager {
public:
    Octomanager();
    ~Octomanager();

    void setStateAndPublish(ExplorationState state);
    void run();

private:
    
    void publishBinaryOctomap();
    void genNeighborCoord(float, float, float);
    void genNeighborCoord(OcTreeKey, std::vector<octomap::point3d>&);

    void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr&);
    void globalPoseCallback(const nav_msgs::Odometry::ConstPtr&);
    void pointReachedCallback(std_msgs::Bool);
    void insertMapScan(const octomap::point3d&, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&);
    void trackChanges(pcl::PointCloud<pcl::PointXYZI>& );
    KeySet find_frontier(pcl::PointCloud<pcl::PointXYZI>&);
    void updateGlobalFrontier(KeySet&);


    void searchForParentsAndPublish(KeySet&);
    void publishParentFrontier();
    void clusterFrontierAndPublish();
    void checkClusteredCells();
    void checkParentCells();
    bool isPointAlreadyAssigned(point3d);
    bool isClusterAlreadyAssigned(point3d);
    bool isClusterObstructed(point3d);
    
    point3d bestCluster(point3d, KeySet& );
    point3d bestParent(point3d);
    void setPointAsInvalid(point3d);

    void keyToPointVector(KeySet& , vector <geometry_msgs::Point>& );
    void pointVectorToKey(vector<geometry_msgs::Point>& ,vector<OcTreeKey>& );

    void addCluster(KeySet&);
    void updateaddClusters();



    void publish_frontier(octomap::KeySet& );    
    void publishClusteredFrontier();
    void publishBestFrontier();
    void publishUAVGoal(point3d);
    void publishAllClusters();

    geometry_msgs::Pose getCurrentUAVPosition(){return uavCurrentPose;} //atengo a esto que no se de donde salio

    bool toggleExplorationServiceCb(std_srvs::SetBool::Request& request, 
			  std_srvs::SetBool::Response& response)
              {
                explorationToggled = request.data;
                if (explorationToggled)
                    cout << "Exploration ON" << endl<<endl;
                else
                    cout << "Exploration OFF" << endl <<endl;
                
                response.success = true;
                response.message = "toggleExplorationService called!";
                return true;
              }

    void filterGroundPlane(
        const pcl::PointCloud<pcl::PointXYZ>::ConstPtr&,
        pcl::PointCloud<pcl::PointXYZ>::Ptr&,
        pcl::PointCloud<pcl::PointXYZ>::Ptr&);
    visualization_msgs::MarkerArray getOccupiedNodesAsMarkers(octomap::OcTree*, const ros::Time&);
    // bool checkObstaclesSrv(octomanager::CheckObstacles::Request&, octomanager::CheckObstacles::Response&);
    // void setInflationSphere();
    // void inflateOccupiedNodes();
    bool isSpeckleNode(const octomap::OcTreeKey& key) const;
    static std_msgs::ColorRGBA heightMapColor(double h);

    inline static void updateMinKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& min)
    {
        for (unsigned i = 0; i < 3; ++i)
            min[i] = std::min(in[i], min[i]);
    };
    inline static void updateMaxKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& max)
    {
        for (unsigned i = 0; i < 3; ++i)
            max[i] = std::max(in[i], max[i]);
    };

    ros::NodeHandle _nh;

    ros::Publisher _octomap_pub;
    ros::Publisher _binaryMapPub;
    ros::Publisher _uavGoalPub;
    ros::Publisher _frontier_pub;
    ros::Publisher _parent_pub;
    ros::Publisher _cluster_pub;
    ros::Publisher _bestFrontierPub;
    ros::Publisher _pubEsmState;
    ros::Publisher _allclusters_pub;

    ros::Subscriber _lidar_sub;
    ros::Subscriber _uavGlobalPoseSub;
    ros::Subscriber _pointReachedSub;

    
    ros::ServiceServer _check_obstacles_srv;
    ros::ServiceServer _serviceExploration;
    ExplorationState _currentState = ExplorationState::OFF;

    tf2_ros::Buffer _buffer;
    tf2_ros::TransformListener _listener;

    Parameters _params;

    octomap::OcTree* _octree; //Creating the class of OcTree, its a pointer to the class. 
    // octomap::OcTree* _inflated_octree;
    octomap::KeyRay _keyRay;
    octomap::OcTreeKey _updateBBXMin;
    octomap::OcTreeKey _updateBBXMax;
    octomap::OcTreeLUT lut{16};
    std::vector<Eigen::Vector3d> _sphere;

    octomap::KeySet globalFrontierCells;
    octomap::KeySet parentFrontierCells; 
    octomap::KeySet parentFrontierCellsUpdated;

    octomap::KeySet candidate_cells;
    octomap::KeySet invalidParentCells;
    octomap::KeySet clusteredCells;
    octomap::KeySet clusteredCellsUpdate;
    octomap::KeyRay keyRaysphere;

    pcl::PointCloud<pcl::PointXYZI> _changedCells;

    best_frontier::BestFrontier bestFrontierServer;
    point3d bestFrontierPoint;
    vector<point3d> allUAVGoals;
    vector<point3d> allClusters;
    vector<point3d> allClustersUpdated;
    geometry_msgs::Pose uavCurrentPose;
    
    bool currentGoalReached {true};
    bool explorationToggled {false};
    unsigned explorationDepth{12};
    float explorationRadius{9};

    double _colorFactor{0.8};
    double _occupancyMinZ{-std::numeric_limits<double>::max()};
    double _occupancyMaxZ{std::numeric_limits<double>::max()};
};
} // namespace catec

