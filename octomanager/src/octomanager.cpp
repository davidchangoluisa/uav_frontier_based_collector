#include <iostream>
#include <octomanager/octomanager.h>
#include <geometry_msgs/TransformStamped.h>
#include <queue>

#include <typeinfo>
#define INF 0x7fffffff

using namespace octomap;
using namespace std;
namespace catec {
Octomanager::Octomanager() : _nh("~"), _listener(_buffer), _octree(NULL)
{
    ROS_INFO("[Octomanager::Octomanager] Starting node...");

    _octree = new octomap::OcTree(_params.resolution);
    _octree->setProbHit(_params.prob_hit);
    _octree->setProbMiss(_params.prob_miss);
    _octree->setClampingThresMin(_params.clamping_thresh_min);
    _octree->setClampingThresMax(_params.clamping_thresh_max);

    _octomap_pub =
        _nh.advertise<visualization_msgs::MarkerArray>(_params.octomap_vis_topic, 1, !_params.pcd_path.empty());

    _frontier_pub    = _nh.advertise<visualization_msgs::MarkerArray>("frontier_cells_vis_array",1,false);
    _parent_pub      = _nh.advertise<visualization_msgs::MarkerArray>("parent_cells_vis_array",1,false);
    _cluster_pub     = _nh.advertise<visualization_msgs::MarkerArray>("cluster_cells_vis_array",1,false);
    _bestFrontierPub = _nh.advertise<visualization_msgs::Marker>("best_frontier_marker", 1, false);
    _binaryMapPub    = _nh.advertise<Octomap>("octomap_binary",1,false);
    _uavGoalPub      = _nh.advertise<geometry_msgs::PoseStamped>("/uav1/exploration/goal", 1, false);
    _pubEsmState     = _nh.advertise<std_msgs::Int32>("exploration/state",1);
    _allclusters_pub = _nh.advertise<visualization_msgs::Marker>("allclusters_vis_array",10);

    _uavGlobalPoseSub   = _nh.subscribe(_params.odom_topic, 1, &Octomanager::globalPoseCallback, this);
    _pointReachedSub    = _nh.subscribe("exploration/point_reached",1,&Octomanager::pointReachedCallback, this);
    _serviceExploration = _nh.advertiseService("exploration/toggle", &Octomanager::toggleExplorationServiceCb,this);

    if (_params.pcd_path.empty()) {
        ROS_INFO("[Octomanager::Octomanager] Sensor mode activated!");
        _lidar_sub = _nh.subscribe(_params.lidar_topic, 1, &Octomanager::pointcloudCallback, this);
    } else {
        ROS_INFO("[Octomanager::Octomanager] Map mode activated!");
    }
    ROS_INFO("[Octomanager::Octomanager] Node ready!");
}

Octomanager::~Octomanager() // To destroy the pointer
{
    _lidar_sub.shutdown();
    _octomap_pub.shutdown();
    _nh.shutdown();

    if (_octree) {
        delete _octree;
        _octree = NULL;
    }

}

void Octomanager::keyToPointVector(KeySet& frontierCells, 
    vector <geometry_msgs::Point>& originalPointsVector)
{
    for(KeySet::iterator iter = frontierCells.begin(), end = frontierCells.end();
        iter!= end; ++iter)
    {
            OcTreeKey tempCell;
            tempCell = *iter;

            point3d tempCellCoordinates;
            tempCellCoordinates = _octree->keyToCoord(tempCell);

            geometry_msgs::Point tempCellPoint;
            tempCellPoint.x = tempCellCoordinates.x();
            tempCellPoint.y = tempCellCoordinates.y();
            tempCellPoint.z = tempCellCoordinates.z();

            originalPointsVector.push_back(tempCellPoint);
    }
}

void Octomanager::pointVectorToKey(vector<geometry_msgs::Point>& points,
		vector<OcTreeKey>& clusterCellsKey)
	{
		for (int i = 0; i < points.size(); i++)
		{
			point3d tempCellCoordinates;
			tempCellCoordinates.x() = points[i].x;
			tempCellCoordinates.y() = points[i].y;
			tempCellCoordinates.z() = points[i].z;
			// Transform from point to key
			OcTreeKey tempCellKey;
			if (!_octree->coordToKeyChecked(tempCellCoordinates, tempCellKey)) 
			{
				OCTOMAP_ERROR_STR("Error in search: [" 
					<< tempCellCoordinates << "] is out of OcTree bounds!");
				return;
			} 
			clusterCellsKey.push_back(tempCellKey);
		}
}

void Octomanager::run()
{
        ros::Rate loopRate(1);
        point3d origin_pose(0,0,0);
        setPointAsInvalid(origin_pose);
        allUAVGoals.push_back(origin_pose);
        cout<<"Origin pose has been deleted"<<endl;
    while (ros::ok())
        {
            ros::spinOnce();
            
            publishBinaryOctomap();
            switch (_currentState)
            {
                case ExplorationState::OFF:
                    {if(explorationToggled)
                        setStateAndPublish(ExplorationState::CHECKFORFRONTIERS);

                    break;}
                
                case ExplorationState::CHECKFORFRONTIERS:
                    uavCurrentPose = getCurrentUAVPosition();
                    cout<<endl<<"==UAVGoals "<<allUAVGoals.size()<<endl;
                    if(!currentGoalReached)
						ROS_WARN_STREAM_THROTTLE(3.0,
						bestFrontierPoint.x() << " " << bestFrontierPoint.y() << " " 
						<< bestFrontierPoint.z() << " -> Goal published!");
                    
                    _octree -> enableChangeDetection(true);
                    _changedCells.clear();
                    trackChanges(_changedCells);
                    if (_changedCells.size()>0)
                        setStateAndPublish(ExplorationState::ON);
                    break;

                case ExplorationState::ON:
                    {
                    
                    globalFrontierCells = find_frontier(_changedCells);
                    searchForParentsAndPublish(globalFrontierCells);
                    addCluster(parentFrontierCellsUpdated);
                    publishAllClusters();
                    
                    if (allClustersUpdated.size()==0)
                        cout << "EXPLORATION FINISHED" << endl;
                    if (!allClustersUpdated.size()>0)
                        setStateAndPublish(ExplorationState::CHECKFORFRONTIERS);
                    else if (isClusterObstructed(bestFrontierPoint))
                    {std_msgs::Int32 exploration_state;
                    exploration_state.data = 77;
                    _pubEsmState.publish(exploration_state);
                    setStateAndPublish(ExplorationState::POINTREACHED);}
                    else if (currentGoalReached)
                        setStateAndPublish(ExplorationState::POINTREACHED);
                    else
                        setStateAndPublish(ExplorationState::CHECKFORFRONTIERS);
                    break;}
                
                case ExplorationState::POINTREACHED:
                    
                    currentGoalReached = false;
                    // UAVMoving  = false;
                    cout << setprecision(2)<<fixed;
                    point3d currentPoint3d(uavCurrentPose.position.x,uavCurrentPose.position.y, uavCurrentPose.position.z);
                    cout << "UAV is at-> (" << uavCurrentPose.position.x<<","<<uavCurrentPose.position.y<<")"<<endl;
                    // bestFrontierPoint = bestFrontierServer.bestFrontierInfGain(_octree,currentPoint3d, clusteredCellsUpdate);
                    // bestFrontierPoint = bestCluster(currentPoint3d, clusteredCellsUpdate);
                    bestFrontierPoint = bestParent(currentPoint3d);
                    cout<<"MEJOR IS AT-> ("<<bestFrontierPoint.x()<<","<<bestFrontierPoint.y()<<")"<<endl;
                    
                    allUAVGoals.push_back(bestFrontierPoint);
                    publishBestFrontier();
                    publishUAVGoal(bestFrontierPoint);
                    ros::Duration(0.05).sleep();
                    setStateAndPublish(ExplorationState::CHECKFORFRONTIERS);
                    break;
                
            }
            
        loopRate.sleep();
        }
}

void Octomanager::genNeighborCoord(float x, float y, float z)
{
    point3d point(x,y,z);
    OcTreeKey key; 
    if (!_octree -> coordToKeyChecked(point, key))
        {
        OCTOMAP_ERROR_STR("Error in search: [" << point << "] is out of OcTree bounds!");
	    return;
        }
        std::vector<octomap::point3d> neighbor;
        genNeighborCoord(key, neighbor);
}

void Octomanager::genNeighborCoord(OcTreeKey start_key, std::vector<octomap::point3d>& occupiedNeighbor) 
{
          occupiedNeighbor.clear();
          OcTreeKey neighbor_key;
          for (int i = 0; i < 26; i++) 
          {
            lut.genNeighborKey(start_key, i, neighbor_key);
	        point3d query = _octree->keyToCoord(neighbor_key);
	        occupiedNeighbor.push_back(query);
         }
}

void Octomanager:: globalPoseCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    uavCurrentPose = msg -> pose.pose;
}

void Octomanager::pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
    ros::WallTime startTime = ros::WallTime::now();

    pcl::PointCloud<pcl::PointXYZ>::Ptr src_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *src_cloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl_ros::transformPointCloud(_params.global_frame_id, *src_cloud, *transformed_cloud, _buffer);

    geometry_msgs::TransformStamped sensor_2_world_tf;
    try {
        sensor_2_world_tf = _buffer.lookupTransform(_params.global_frame_id, msg->header.frame_id, msg->header.stamp);
    } catch (tf2::TransformException& ex) {
        ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
        return;
    }

    const octomap::point3d sensor_2_world_p(
        sensor_2_world_tf.transform.translation.x,
        sensor_2_world_tf.transform.translation.y,
        sensor_2_world_tf.transform.translation.z);

    insertMapScan(sensor_2_world_p, transformed_cloud);
    publish_frontier(globalFrontierCells);
    

    double total_elapsed = (ros::WallTime::now() - startTime).toSec();
    ROS_INFO_STREAM("Elapsed time: " << total_elapsed);

    visualization_msgs::MarkerArray occupied_nodes_msg = getOccupiedNodesAsMarkers(_octree, msg->header.stamp);
    _octomap_pub.publish(occupied_nodes_msg);
}


void Octomanager::insertMapScan(
    const octomap::point3d& sensorOrigin, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
{
    if (!_octree->coordToKeyChecked(sensorOrigin, _updateBBXMin) ||
        !_octree->coordToKeyChecked(sensorOrigin, _updateBBXMax)) {
        ROS_ERROR_STREAM("Could not generate Key for origin " << sensorOrigin);
    }

    // instead of direct scan insertion, compute update to filter ground:
    octomap::KeySet free_cells, occupied_cells;

    // all other points: free on ray, occupied on endpoint:
    for (pcl::PointCloud<pcl::PointXYZ>::const_iterator it = cloud->begin(); it != cloud->end(); ++it) {
        octomap::point3d point(it->x, it->y, it->z);
        // cout << point << endl; To visualize the points

        if ((_params.min_sensor_range > 0) && (point - sensorOrigin).norm() < _params.min_sensor_range)
            continue;

        // maxrange check
        if ((_params.max_sensor_range < 0.0) || ((point - sensorOrigin).norm() <= _params.max_sensor_range)) {
            // free cells
            if (_octree->computeRayKeys(sensorOrigin, point, _keyRay)) {
                free_cells.insert(_keyRay.begin(), _keyRay.end());
            }
            // occupied endpoint
            octomap::OcTreeKey key;
            if (_octree->coordToKeyChecked(point, key)) {
                occupied_cells.insert(key);

                updateMinKey(key, _updateBBXMin);
                updateMaxKey(key, _updateBBXMax);
            }
        } else { // ray longer than maxrange:;
            octomap::point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * _params.max_sensor_range;
            if (_octree->computeRayKeys(sensorOrigin, new_end, _keyRay)) {
                free_cells.insert(_keyRay.begin(), _keyRay.end());

                octomap::OcTreeKey endKey;
                if (_octree->coordToKeyChecked(new_end, endKey)) {
                    free_cells.insert(endKey);
                    updateMinKey(endKey, _updateBBXMin);
                    updateMaxKey(endKey, _updateBBXMax);
                } else {
                    ROS_ERROR_STREAM("Could not generate Key for endpoint " << new_end);
                }
            }
        }
    }

    // mark free cells only if not seen occupied in this cloud
    for (octomap::KeySet::iterator it = free_cells.begin(), end = free_cells.end(); it != end; ++it) {
        if (occupied_cells.find(*it) == occupied_cells.end()) {
            _octree->updateNode(*it, false);
        }
    }

    // now mark all occupied cells:
    for (octomap::KeySet::iterator it = occupied_cells.begin(), end = occupied_cells.end(); it != end; it++) {
        _octree->updateNode(*it, true);
    }

    octomap::point3d minPt, maxPt;
    ROS_DEBUG_STREAM(
        "Bounding box keys (before): " << _updateBBXMin[0] << " " << _updateBBXMin[1] << " " << _updateBBXMin[2]
                                       << " / " << _updateBBXMax[0] << " " << _updateBBXMax[1] << " "
                                       << _updateBBXMax[2]);

    minPt = _octree->keyToCoord(_updateBBXMin);
    maxPt = _octree->keyToCoord(_updateBBXMax);
    ROS_DEBUG_STREAM("Updated area bounding box: " << minPt << " - " << maxPt);
    ROS_DEBUG_STREAM(
        "Bounding box keys (after): " << _updateBBXMin[0] << " " << _updateBBXMin[1] << " " << _updateBBXMin[2] << " / "
                                      << _updateBBXMax[0] << " " << _updateBBXMax[1] << " " << _updateBBXMax[2]);

    _octree->prune();
}

void Octomanager::trackChanges(pcl::PointCloud<pcl::PointXYZI>& changedCells)
{
    // pcl::PointCloud<pcl::PointXYZI> changed_cells;
    octomap::KeyBoolMap::const_iterator startPnt = _octree->changedKeysBegin();
    octomap::KeyBoolMap::const_iterator endPnt = _octree->changedKeysEnd();

    int c=0; 
    for (octomap::KeyBoolMap::const_iterator iter = startPnt; iter!=endPnt; ++iter)
    {
        c++; 
        octomap::OcTreeNode* node = _octree ->search(iter->first);
        bool occupied = _octree->isNodeOccupied(node);
        pcl::PointXYZI pnt;
        pnt.x = _octree->keyToCoord(iter->first.k[0]);
        pnt.y = _octree->keyToCoord(iter->first.k[1]);
        pnt.z = _octree->keyToCoord(iter->first.k[2]);
        if (occupied)
        {
            pnt.intensity = 1000;
        }
        else 
        {
            pnt.intensity = -1000;
        }
            changedCells.push_back(pnt);
    }
    _octree -> resetChangeDetection();
}

KeySet Octomanager::find_frontier(pcl::PointCloud<pcl::PointXYZI>& changedCells)
{
    KeySet globalFrontierCells;
    
    bool unknownCellFlag{false};
    bool freeCellFlag{false};

    std::vector <octomap::point3d> neighbor; 
    int frontierSize {0};


    for (int i=0; i<changedCells.points.size(); i++)
    {
        //Check if the new point from changed cells is within the exploration limits
        if(changedCells.points[i].x < _params.bbx_minX ||
            changedCells.points[i].x > _params.bbx_maxX||
            changedCells.points[i].y < _params.bbx_minY ||
            changedCells.points[i].y > _params.bbx_maxY ||
            changedCells.points[i].z < _params.bbx_minZ ||
            changedCells.points[i].z > _params.bbx_maxZ) continue;
        
        // Get changed point
        point3d changedCellPoint(
            changedCells.points[i].x,
            changedCells.points[i].y,
            changedCells.points[i].z);

        OcTreeKey changedCellKey; // How this contain the info of the point? 
        if (!_octree -> coordToKeyChecked(changedCellPoint, changedCellKey))
        {
            OCTOMAP_ERROR_STR("Error in search: [" 
                << changedCellPoint << "] is out of OcTree bounds!");
                return globalFrontierCells; 
        }

        //Check point state: free/occupied
        OcTreeNode* changedCellNode = _octree -> search(changedCellKey);
        bool changedCellOccupied    = _octree -> isNodeOccupied(changedCellNode);

        if (!changedCellOccupied)
        {
            unknownCellFlag = false;
            freeCellFlag = false;
            genNeighborCoord(changedCellKey,neighbor);
            for (std::vector<point3d>::iterator iter = neighbor.begin();iter != neighbor.end(); iter++)
            {
                point3d neipoint =* iter;

                //Check point state: free/unknown
                OcTreeNode* node = _octree-> search(neipoint);
                if (node == NULL)
                    unknownCellFlag = true;
                else if(_octree->isNodeOccupied(node))
                    freeCellFlag = true;
                
            }
            if(unknownCellFlag && freeCellFlag)
            {globalFrontierCells.insert(changedCellKey);}
        }
    }
    return globalFrontierCells;
}   

// void Octomanager:: updateGlobalFrontier(KeySet& globalFrontierCells)
// {
//     int frontierSize{0};
//     FrontierCellsUpdated.clear();
//     for(KeySet::iterator iter = globalFrontierCells.begin(), end = globalFrontierCells.end();
//         iter!= end; ++iter)
//         {
//             frontierSize++;
//         }
//         cout << "Frontiers before " << frontierSize<<endl;

// 		bool unknownCellFlag {false};
// 		bool freeCellFlag {false};
// 		bool occupiedCellFlag {false};
// 		int deleted {0};
        
//         std::vector<octomap::point3d> changedCellNeighbor;

// 		for(KeySet::iterator cell_iter = globalFrontierCells.begin(), end = globalFrontierCells.end();
// 			cell_iter!= end; )
// 		{
// 			// If current cell if free, check its neighbors
// 			unknownCellFlag = false;
// 			occupiedCellFlag = false;
// 			freeCellFlag = false;
// 			genNeighborCoord(*cell_iter, changedCellNeighbor);
// 			for (std::vector<point3d>::iterator neighbor_iter = changedCellNeighbor.begin();
// 				neighbor_iter != changedCellNeighbor.end(); neighbor_iter++)
// 			{

// 				// Check for neighbors
// 				OcTreeNode* node = _octree->search(*neighbor_iter);
// 				if(node == NULL)
//                     {unknownCellFlag = true;}
// 				else if(!_octree->isNodeOccupied(node))
// 					freeCellFlag = true;
// 				else if (_octree->isNodeOccupied(node))
// 					occupiedCellFlag = true;
// 			}
// 			if(!unknownCellFlag || occupiedCellFlag)
// 			{
// 				// globalFrontierCells.erase(*cell_iter);
// 				deleted++;	
// 			}
// 			else FrontierCellsUpdated.insert(*cell_iter);
			
// 			cell_iter++;				
// 		}
//         cout << "Number of deleted frontiers: " << deleted << endl;
//         frontierSize = 0;

// 		for(KeySet::iterator iter = FrontierCellsUpdated.begin(), end = FrontierCellsUpdated.end();
// 			iter!= end; ++iter)
// 		{
// 			frontierSize++;
// 		}
// 		cout << "Number of global frontiers after:" << frontierSize << endl;       
// }

void Octomanager:: clusterFrontierAndPublish()
{
    clusteredCells.clear();

    // Preprocess put the frontier cells into a vector
    std::vector<geometry_msgs::Point> originalPointsVector {};
    std::vector<geometry_msgs::Point> clusteredPointsVector {};

    keyToPointVector(parentFrontierCells, originalPointsVector);
    MSCluster *cluster = new MSCluster();
    cluster->getMeanShiftClusters(
        originalPointsVector, clusteredPointsVector, _params.kernel_bandwidth);
    vector<OcTreeKey> clusterCellsKey {};
    pointVectorToKey(clusteredPointsVector, clusterCellsKey);
    for (std::vector<OcTreeKey>::iterator iter = clusterCellsKey.begin();
        iter != clusterCellsKey.end(); iter++)
        {
            clusteredCells.insert(*iter);
        }
    
    delete cluster;
    cout << "cluster_size" << clusteredCells.size() << endl;
    checkClusteredCells();
    publishClusteredFrontier();
}

void Octomanager::checkClusteredCells()
{
    int deletedNum{0};
    clusteredCellsUpdate.clear();
    for(KeySet::iterator iter = clusteredCells.begin(), 
        end = clusteredCells.end(); iter != end; ++iter)
        {
           point3d tempCellPosition = _octree->keyToCoord(*iter);
           if (isPointAlreadyAssigned(tempCellPosition))
           {
                deletedNum++;
           }
           else
           {
                clusteredCellsUpdate.insert(*iter);
           }
        //    cout << "--***--**--Deleted candidates num: "<< deletedNum <<endl;

        }
}

bool Octomanager::isPointAlreadyAssigned(point3d point)
{
    for (int i = 0; i < allUAVGoals.size(); i++)
    {
        if (
        fabs(allUAVGoals[i].x() - point.x()) < explorationRadius &&
        fabs(allUAVGoals[i].y() - point.y()) < explorationRadius &&
        fabs(allUAVGoals[i].z() - point.z()) < explorationRadius)
        {
            // Point is too close to be assigned 
            // ROS_WARN("Similar point has been already assigned!");
            return true;
        }	
    }
    return false;
}

void Octomanager::searchForParentsAndPublish(KeySet& globalFrontierCells)
{
    //Search in globalFrontierCells after update
    cout << "finding parents"<<endl;
    for(KeySet::iterator iter = globalFrontierCells.begin(), end = globalFrontierCells.end(); iter != end; ++iter)
    {
        OcTreeNode* parentNodePtr = _octree->search(*iter, explorationDepth);
        parentNodePtr->setValue(-1);
    }

    // Iter over desire depth and ask if the node is frontier

    int counter{0};
    parentFrontierCells.clear();

    for (OcTree::iterator it = _octree->begin(explorationDepth), end = _octree->end(); it != end; ++it)
    {
        if(it->getValue() == -1)
        {
            auto found =  invalidParentCells.find(it.getKey());
            if (found != invalidParentCells.end())
            {
                continue;
            }
            else
            {  
                parentFrontierCells.insert(it.getKey());
                counter++;
            }

        }
    }
    checkParentCells();
    cout << "FrontierServer - parents number: " << counter << endl;

    publishParentFrontier();
}

void Octomanager::checkParentCells()
{
    int deletedNum{0};
    parentFrontierCellsUpdated.clear();
    for(KeySet::iterator iter = parentFrontierCells.begin(), 
        end = parentFrontierCells.end(); iter != end; ++iter)
        {
           point3d tempCellPosition = _octree->keyToCoord(*iter);
           if (isPointAlreadyAssigned(tempCellPosition))
           {
                deletedNum++;
           }
           else
           {
                parentFrontierCellsUpdated.insert(*iter);
           }
        }
}


point3d Octomanager::bestCluster(point3d cur_p, KeySet& Cells)
{
    float min_a = INF;
    float wei_l, wei_i, wei_a; //l=longitud, i=ratio of cells, a=total
    int numall = 0, numunknown =0; //#all cells and #unknown cells
    int i;
    point3d best_goal;
    for (KeySet::iterator iter = Cells.begin(), end=Cells.end(); iter!=end;++iter)
    {
        point3d fpoint;
        fpoint = _octree->keyToCoord(*iter);
        wei_l=(fpoint.x()-cur_p.x())*(fpoint.x()-cur_p.x())+(fpoint.y()-cur_p.y())*(fpoint.y()-cur_p.y())+(fpoint.z()-cur_p.z())*(fpoint.z()-cur_p.z()); //term that consideres the distance
        point3d sphere_point[6] ;
        for (int j=0; j<6; j++)
            sphere_point[j] = fpoint;
        sphere_point[0].x() += _params.max_sensor_range;
        sphere_point[1].x() -= _params.max_sensor_range;
        sphere_point[2].y() += _params.max_sensor_range;
        sphere_point[3].y() -= _params.max_sensor_range;
        sphere_point[4].z() += _params.max_sensor_range;
        sphere_point[5].z() -= _params.max_sensor_range;
        numall = 0, numunknown =0;
        for (int j=0; j<6; j++)
        {
            _octree->computeRayKeys(fpoint, sphere_point[j],keyRaysphere);
            for(KeyRay::iterator it = keyRaysphere.begin(); it != keyRaysphere.end(); it++)
            {
                numall++;
            OcTreeNode* node = _octree->search(*it);
            if(node ==NULL)
                numunknown++;
            }
        }
        wei_i = numunknown/(numall*1.0); // term that consideres the ratio of cells
        // wei_a = 0.8*wei_i + 0.2*wei_l;
        wei_a = wei_l; //by doing this it will consider the closest always
        if (wei_a < min_a)
        {
            min_a = wei_a;
            best_goal = fpoint;
        }
    }
    
    return best_goal;
}

point3d Octomanager::bestParent(point3d cur_p)
{
    float min_a = INF;
    float w_dist; //l=longitud, i=ratio of cells, a=total
    int numall = 0, numunknown =0; //#all cells and #unknown cells
    int i;
    point3d best_parent;
    for (int i=0;  i < allClustersUpdated.size(); i++)
    {
        point3d fpoint;
        fpoint = allClustersUpdated[i];
        w_dist=(fpoint.x()-cur_p.x())*(fpoint.x()-cur_p.x())+(fpoint.y()-cur_p.y())*(fpoint.y()-cur_p.y())+(fpoint.z()-cur_p.z())*(fpoint.z()-cur_p.z()); //term that consideres the distance

        if (w_dist < min_a)
        {
            min_a = w_dist;
            best_parent = fpoint;
        }
    }
    
    return best_parent;
}


void Octomanager::addCluster(KeySet& Cells)
{

    for (KeySet::iterator iter = Cells.begin(), end=Cells.end(); iter!=end;++iter)
    {
        point3d fpoint;
        fpoint = _octree->keyToCoord(*iter);

        if (isClusterAlreadyAssigned(fpoint))
        {
            continue;
        }
        else
        {
            allClusters.push_back(fpoint);
        }  
        
    }
    updateaddClusters();
    
}


void Octomanager::updateaddClusters()
{   

    allClustersUpdated.clear();
    for(int i=0; i<allClusters.size(); i++)
    {
        if (isPointAlreadyAssigned(allClusters[i]) || isClusterObstructed(allClusters[i]))
        {
            cout<<"UN PUNTO ELIMINA0.."<<endl;
            continue;
        }
        else;
        {allClustersUpdated.push_back(allClusters[i]);
        }
    }
    cout<<"Me faltan agregar: "<<allClustersUpdated.size()<<" Parents"<<endl;

}



bool Octomanager::isClusterAlreadyAssigned(point3d point)
{
    for (int i = 0; i < allClusters.size(); i++)
    {
        if (
        fabs(allClusters[i].x() - point.x()) < explorationRadius &&
        fabs(allClusters[i].y() - point.y()) < explorationRadius &&
        fabs(allClusters[i].z() - point.z()) < explorationRadius)
        {
            return true;
            cout<<"*****Doble eliminado.."<<endl;
        }	
    }
    return false;
}

bool Octomanager::isClusterObstructed(point3d point)
{
    bool unknownCellFlag {false};
    bool freeCellFlag {false};
    bool occupiedCellFlag {false};
    OcTreeKey key;
    vector<point3d> vecinos;
    _octree -> coordToKeyChecked(point, key);
    genNeighborCoord(key, vecinos);
    for (std::vector<point3d>::iterator veci_iter = vecinos.begin();veci_iter !=vecinos.end(); veci_iter++ )
    {
        OcTreeNode* node = _octree->search(*veci_iter);
        if(node == NULL)
            {unknownCellFlag = true;}
        else if(!_octree->isNodeOccupied(node))
            {freeCellFlag = true;
            }
        else if (_octree->isNodeOccupied(node))
            {occupiedCellFlag = true;
            }
    }
    if(occupiedCellFlag)
    {
        cout<<"encontre algo"<<endl;
        setPointAsInvalid(point);
        return true;
    }
    else return false;
}

void Octomanager::publishParentFrontier()
{
    visualization_msgs::MarkerArray ParentNodesVis;
    ParentNodesVis.markers.resize(_octree->getTreeDepth()  + 1);

    int counter{0};
    for (OcTree::iterator it = _octree->begin(_octree->getTreeDepth() ),end = _octree->end(); it != end; ++it)
    {
        bool isfron = false;
        for(KeySet::iterator iter = parentFrontierCells.begin(), end = parentFrontierCells.end(); iter!= end; ++iter)
        {
            octomap::point3d fpoint;
            fpoint = _octree->keyToCoord(*iter);
            				
            if (fabs(it.getX() - fpoint.x()) <= _params.resolution /2 &&
                fabs(it.getY() - fpoint.y()) <= _params.resolution /2 &&
                fabs(it.getZ() - fpoint.z()) <= _params.resolution /2)
            {
                isfron = true;
                counter++;
            }

        }
        if (isfron)
        {
            double x = it.getX();
            double y = it.getY();
            double z = it.getZ();

            unsigned idx = it.getDepth();
            assert(idx < ParentNodesVis.markers.size());

            geometry_msgs::Point cubeCenter;
            cubeCenter.x = x;
            cubeCenter.y = y;
            cubeCenter.z = z;
            ParentNodesVis.markers[idx].points.push_back(cubeCenter);
        }
    }

    std_msgs::ColorRGBA colorFrontier;
    colorFrontier.r = 1.0;
    colorFrontier.g = 0.0;
    colorFrontier.b = 0.0;
    colorFrontier.a = 1.0;
    for (unsigned i= 0; i < ParentNodesVis.markers.size(); ++i)
    {
        double size = _octree->getNodeSize(i);
        ParentNodesVis.markers[i].header.frame_id = _params.global_frame_id;
        ParentNodesVis.markers[i].header.stamp = ros::Time::now();
        ParentNodesVis.markers[i].ns = "map";
        ParentNodesVis.markers[i].id = i;
        ParentNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
        ParentNodesVis.markers[i].scale.x = size;
        ParentNodesVis.markers[i].scale.y = size;
        ParentNodesVis.markers[i].scale.z = size;
        ParentNodesVis.markers[i].color = colorFrontier;

        if (ParentNodesVis.markers[i].points.size() > 0)
            ParentNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
        else
           ParentNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }

    _parent_pub.publish(ParentNodesVis);
}



void Octomanager::publish_frontier(KeySet& frontierCells)
{
    visualization_msgs::MarkerArray frontierNodesVis;
    frontierNodesVis.markers.resize(_octree->getTreeDepth() +1);

    for (octomap::OcTree::iterator it = _octree->begin(_octree->getTreeDepth()), end = _octree->end(); it != end; ++it)
    {
        bool isfron = false; 
        for (KeySet::iterator iter = frontierCells.begin(), end=frontierCells.end(); iter!=end; ++iter)
        {
            octomap::point3d fpoint;
            fpoint = _octree->keyToCoord(*iter);
            if(it.getX() == fpoint.x() && it.getY() == fpoint.y() && it.getZ() == fpoint.z() )
                isfron = true;
        }
        if (isfron)
        {
            double x = it.getX();
            double y = it.getY();
            double z = it.getZ();

            unsigned idx = it.getDepth();
            assert(idx < frontierNodesVis.markers.size());

            geometry_msgs:: Point cubeCenter; 
            cubeCenter.x = x; 
            cubeCenter.y = y; 
            cubeCenter.z = z; 

            frontierNodesVis.markers[idx].points.push_back(cubeCenter);
        }
    }
    for (unsigned i=0; i<frontierNodesVis.markers.size();i++)
    {

        double size = _octree -> getNodeSize(i);
        frontierNodesVis.markers[i].header.frame_id = _params.global_frame_id;
        frontierNodesVis.markers[i].header.stamp = ros::Time::now();
        frontierNodesVis.markers[i].ns = "map";
        frontierNodesVis.markers[i].id = i;
        frontierNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
        frontierNodesVis.markers[i].scale.x = size;
        frontierNodesVis.markers[i].scale.y = size;
        frontierNodesVis.markers[i].scale.z = size;
        frontierNodesVis.markers[i].color.r = 1;
        frontierNodesVis.markers[i].color.g = 0;
        frontierNodesVis.markers[i].color.b = 0;
        frontierNodesVis.markers[i].color.a = 1;

        if (frontierNodesVis.markers[i].points.size() > 0)
            frontierNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
        else
            frontierNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
        
    }

    _frontier_pub.publish(frontierNodesVis);

}


void Octomanager::publishClusteredFrontier()
{
    		// init markers for free space:
    visualization_msgs::MarkerArray ClusterNodesVis;
    // each array stores all cubes of a different size, one for each depth level:
    ClusterNodesVis.markers.resize(_octree->getTreeDepth() +1);

    for (octomap::OcTree::iterator it = _octree->begin(_octree->getTreeDepth()), end = _octree->end(); it != end; ++it)
    {
        bool isfron = false;
        for(KeySet::iterator iter = clusteredCellsUpdate.begin(), end = clusteredCellsUpdate.end(); iter!= end; ++iter)
        {
            octomap::point3d fpoint;
            fpoint = _octree->keyToCoord(*iter);
            if (fabs(it.getX() - fpoint.x()) <= _params.resolution /2 &&
                fabs(it.getY() - fpoint.y()) <= _params.resolution /2 &&
                fabs(it.getZ() - fpoint.z()) <= _params.resolution /2) isfron = true;
        }
        if (isfron)
        {
            double x = it.getX();
            double y = it.getY();
            double z = it.getZ();

            unsigned idx = it.getDepth();
            assert(idx < ClusterNodesVis.markers.size());

            geometry_msgs::Point cubeCenter;
            cubeCenter.x = x;
            cubeCenter.y = y;
            cubeCenter.z = z;

            ClusterNodesVis.markers[idx].points.push_back(cubeCenter);
        } 
    }
    // finish MarkerArray:
    std_msgs::ColorRGBA colorClusteredFrontier;
    colorClusteredFrontier.r = 1.0;
    colorClusteredFrontier.g = 0.9;
    colorClusteredFrontier.b = 0;
    colorClusteredFrontier.a = 1;

    for (unsigned i= 0; i < ClusterNodesVis.markers.size(); ++i)
    {
        double size = _octree->getNodeSize(i);

        ClusterNodesVis.markers[i].header.frame_id = _params.global_frame_id;
        ClusterNodesVis.markers[i].header.stamp = ros::Time::now();
        ClusterNodesVis.markers[i].ns = "map";
        ClusterNodesVis.markers[i].id = i;
        ClusterNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
        ClusterNodesVis.markers[i].scale.x = size;
        ClusterNodesVis.markers[i].scale.y = size;
        ClusterNodesVis.markers[i].scale.z = size;
        ClusterNodesVis.markers[i].color = colorClusteredFrontier;

        if (ClusterNodesVis.markers[i].points.size() > 0)
            ClusterNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
        else
            ClusterNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }
    _cluster_pub.publish(ClusterNodesVis);
}


void Octomanager::setStateAndPublish(ExplorationState state)
{
    _currentState = state;
    std_msgs::Int32 stateMsg;
    stateMsg.data = _currentState;
    _pubEsmState.publish(stateMsg);
    ROS_INFO_STREAM("UpdateStatus - state activated: " << ToString(_currentState));
}


void Octomanager::pointReachedCallback(std_msgs::Bool msg)
{
    if (msg.data)
    {
        currentGoalReached = true;
        cout << "<<<<<<_____ENTRE" <<endl;
    }
}

visualization_msgs::MarkerArray
Octomanager::getOccupiedNodesAsMarkers(octomap::OcTree* octree, const ros::Time& rostime)
{
    ros::WallTime startTime = ros::WallTime::now();
    size_t octomapSize      = octree->size();
    // init markers:
    visualization_msgs::MarkerArray occupiedNodesVis;

    if (octomapSize <= 1) {
        ROS_WARN("Nothing to publish, octree is empty");
        return occupiedNodesVis;
    }

    // each array stores all cubes of a different size, one for each depth level:
    occupiedNodesVis.markers.resize(octree->getTreeDepth() + 1);

    // traverse all leafs in the tree:
    for (octomap::OcTree::iterator it = octree->begin(octree->getTreeDepth()), end = octree->end(); it != end; ++it) {
        if (octree->isNodeOccupied(*it)) {
            double z         = it.getZ();
            double half_size = it.getSize() / 2.0;
            if (z + half_size > _occupancyMinZ && z - half_size < _occupancyMaxZ) {
                double x = it.getX();
                double y = it.getY();
                // ROS_DEBUG_STREAM("Occupied point: " << x << "-"<<y << "-"<<z);
                // ROS_DEBUG_STREAM("half size: " << half_size);

                // create marker:
                unsigned idx = it.getDepth();
                assert(idx < occupiedNodesVis.markers.size());

                geometry_msgs::Point cubeCenter;
                cubeCenter.x = x;
                cubeCenter.y = y;
                cubeCenter.z = z;

                occupiedNodesVis.markers[idx].points.push_back(cubeCenter);

                double minX, minY, minZ, maxX, maxY, maxZ;
                octree->getMetricMin(minX, minY, minZ);
                octree->getMetricMax(maxX, maxY, maxZ);

                double h = (1.0 - std::min(std::max((cubeCenter.z - minZ) / (maxZ - minZ), 0.0), 1.0)) * _colorFactor;
                occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(h));
            }
        }
    }

    // finish MarkerArray:
    for (unsigned i = 0; i < occupiedNodesVis.markers.size(); ++i) {
        double size = octree->getNodeSize(i);

        occupiedNodesVis.markers[i].header.frame_id = _params.global_frame_id;
        occupiedNodesVis.markers[i].header.stamp    = rostime;
        occupiedNodesVis.markers[i].ns              = "map";
        occupiedNodesVis.markers[i].id              = i;
        occupiedNodesVis.markers[i].type            = visualization_msgs::Marker::CUBE_LIST;
        occupiedNodesVis.markers[i].scale.x         = size;
        occupiedNodesVis.markers[i].scale.y         = size;
        occupiedNodesVis.markers[i].scale.z         = size;

        if (occupiedNodesVis.markers[i].points.size() > 0)
            occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
        else
            occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }

    double total_elapsed = (ros::WallTime::now() - startTime).toSec();
    ROS_DEBUG_STREAM("Map publishing took " << total_elapsed << " sec");

    return occupiedNodesVis;
}


void Octomanager::publishBestFrontier()
{
    cout << "********BEST is at-> (" << bestFrontierPoint.x()<<","<<bestFrontierPoint.y()<<","<<bestFrontierPoint.z()<<")"<<endl;
    

    visualization_msgs::Marker frontier_goal;
    std_msgs::ColorRGBA colorgoalFrontier;
    colorgoalFrontier.r = 1;
    colorgoalFrontier.g = 0.5;
    colorgoalFrontier.b = 1;
    colorgoalFrontier.a = 1;
    geometry_msgs::Point cubeCenter;
    cubeCenter.x = bestFrontierPoint.x();
    cubeCenter.y = bestFrontierPoint.y();
    cubeCenter.z = bestFrontierPoint.z();

    frontier_goal.points.push_back(cubeCenter);
    double size = (_octree->getNodeSize(_octree->getTreeDepth()))*3;
    frontier_goal.header.frame_id = _params.global_frame_id;
    frontier_goal.header.stamp = ros::Time::now();
    frontier_goal.ns = "map";
    frontier_goal.type = visualization_msgs::Marker::SPHERE_LIST;
    frontier_goal.scale.x = size;
    frontier_goal.scale.y = size;
    frontier_goal.scale.z = size;
    frontier_goal.color = colorgoalFrontier;
    if (frontier_goal.points.size() > 0)
        frontier_goal.action = visualization_msgs::Marker::ADD;
    else
        frontier_goal.action = visualization_msgs::Marker::DELETE;

    _bestFrontierPub.publish(frontier_goal);
}

std_msgs::ColorRGBA Octomanager::heightMapColor(double h)
{
    std_msgs::ColorRGBA color;
    color.a = 1;
    // blend over HSV-values (more colors)

    double s = 1.0;
    double v = 1.0;

    h -= floor(h);
    h *= 6;
    int i;
    double m, n, f;

    i = floor(h);
    f = h - i;
    if (!(i & 1))
        f = 1 - f; // if i is even
    m = v * (1 - s);
    n = v * (1 - s * f);

    switch (i) {
        case 6:
        case 0:
            color.r = v;
            color.g = n;
            color.b = m;
            break;
        case 1:
            color.r = n;
            color.g = v;
            color.b = m;
            break;
        case 2:
            color.r = m;
            color.g = v;
            color.b = n;
            break;
        case 3:
            color.r = m;
            color.g = n;
            color.b = v;
            break;
        case 4:
            color.r = n;
            color.g = m;
            color.b = v;
            break;
        case 5:
            color.r = v;
            color.g = m;
            color.b = n;
            break;
        default:
            color.r = 1;
            color.g = 0.5;
            color.b = 0.5;
            break;
    }

    return color;
}


void Octomanager::filterGroundPlane(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& map,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& ground,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& nonground)
{
    ground->header    = map->header;
    nonground->header = map->header;

    pcl::ModelCoefficients::Ptr plane(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setAxis(Eigen::Vector3f::UnitZ());
    seg.setEpsAngle(pcl::deg2rad(0.0));
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(_params.ground_removal_dist);
    seg.setInputCloud(map);
    seg.setMaxIterations(70);
    seg.segment(*inliers, *plane);

    pcl::ExtractIndices<pcl::PointXYZ> idx_extr;
    idx_extr.setInputCloud(map);
    idx_extr.setIndices(inliers);
    idx_extr.setNegative(false);
    idx_extr.filter(*ground);
    idx_extr.setNegative(true);
    idx_extr.filter(*nonground);
}



bool Octomanager::isSpeckleNode(const octomap::OcTreeKey& key) const
{
    octomap::OcTreeKey current_key;
    for (current_key[2] = key[2] - 1; current_key[2] <= key[2] + 1; ++current_key[2]) {
        for (current_key[1] = key[1] - 1; current_key[1] <= key[1] + 1; ++current_key[1]) {
            for (current_key[0] = key[0] - 1; current_key[0] <= key[0] + 1; ++current_key[0]) {
                if (current_key != key) {
                    octomap::OcTreeNode* node = _octree->search(current_key);
                    if (node && _octree->isNodeOccupied(node))
                        return false;
                }
            }
        }
    }
    return true;
}

void Octomanager:: publishBinaryOctomap()
{
    Octomap map;
    map.header.frame_id = _params.global_frame_id;
    map.header.stamp    = ros::Time::now();

    if(octomap_msgs::binaryMapToMsg(*_octree, map))
        _binaryMapPub.publish(map);
    else
        ROS_ERROR("Error serializing Octomap");
}

void Octomanager::setPointAsInvalid(point3d point)
{
    cout <<  "Setting point: " << point << " as invalid" << endl;
    OcTreeKey invalidCellKey;
    if (!_octree->coordToKeyChecked(point, invalidCellKey))
    {
			OCTOMAP_ERROR_STR("Error in search: [" 
				<< point << "] is out of OcTree bounds!");
			return;
    }
    invalidParentCells.insert(invalidCellKey);
}

void Octomanager::publishUAVGoal(point3d goal)
{
        
		if (goal.x() < _params.bbx_minX || 
			goal.x() > _params.bbx_maxX ||
			goal.y() < _params.bbx_minY || 
			goal.y() > _params.bbx_maxY ||
			goal.z() < _params.bbx_minZ || 
			goal.z() > _params.bbx_maxZ) 
		{
			ROS_ERROR("Want to publish a goal out of the bounding box.");
			setPointAsInvalid(goal);
			currentGoalReached = true;
			return; 
		}

        if (isClusterObstructed(goal))
        {
            currentGoalReached = true;
            ROS_ERROR("The goal is unreachable");
			setPointAsInvalid(goal);
            return;
        }	        

        // UAVMoving = true; 
        
        geometry_msgs::PoseStamped m_goal;

		m_goal.header.frame_id = _params.global_frame_id;
		m_goal.header.stamp = ros::Time::now();

		m_goal.pose.position.x = goal.x();
		m_goal.pose.position.y = goal.y();
		m_goal.pose.position.z = goal.z();
		m_goal.pose.orientation.x = 0;
		m_goal.pose.orientation.y = 0;
		m_goal.pose.orientation.z = 0;
		m_goal.pose.orientation.w = 1;
        

		_uavGoalPub.publish(m_goal);
		ROS_WARN_STREAM(goal.x() << " " << goal.y() << " " << goal.z() << " -> Goal TOPIC published!");
}

void Octomanager::publishAllClusters()
{

    visualization_msgs::Marker AllClustersVis;
    AllClustersVis.header.frame_id = _params.global_frame_id;
    AllClustersVis.header.stamp = ros::Time::now();
    AllClustersVis.ns = "Allclusters";
    AllClustersVis.action = visualization_msgs::Marker::ADD;
    AllClustersVis.pose.orientation.w = 1.0;
    AllClustersVis.id = 0;
    AllClustersVis.type = visualization_msgs::Marker::SPHERE_LIST;
    AllClustersVis.scale.x = 1;
    AllClustersVis.scale.y = 1;
    AllClustersVis.scale.z = 1;

    AllClustersVis.color.g = 1.0f;
    AllClustersVis.color.a = 1.0;
    for (uint32_t i=0; i<allClustersUpdated.size(); i++)
    {
        geometry_msgs:: Point p;
        p.x = allClustersUpdated[i].x();
        p.y = allClustersUpdated[i].y();
        p.z = allClustersUpdated[i].z();
        AllClustersVis.points.push_back(p);
    }

    _allclusters_pub.publish(AllClustersVis);




    // visualization_msgs::MarkerArray AllClustersVis;

    // std_msgs::ColorRGBA colorAllClusters;
    // colorAllClusters.r = 0;
    // colorAllClusters.g = 1;
    // colorAllClusters.b = 0;
    // colorAllClusters.a = 1;

    // for (int i = 0; i<allClusters.size(); i++)
    // {
    //     geometry_msgs::Point clusterCenter;
    //     clusterCenter.x = allClusters[i].x();
    //     clusterCenter.y = allClusters[i].y();
    //     clusterCenter.z = allClusters[i].z();

    //     AllClustersVis.markers[i].points.push_back(clusterCenter);
    //     cout<<"Cluster: "<<i<<allClusters[i].x()<<endl;
    // }
    
    // for(unsigned i=0; i<AllClustersVis.markers.size(); i++)
    // {
    //     AllClustersVis.markers[i].header.frame_id = _params.global_frame_id;
    //     AllClustersVis.markers[i].header.stamp = ros::Time::now();
    // }




}

} // namespace catec
