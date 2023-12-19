#include "exploration.h"

using namespace octomap;
using namespace std;

FrontierExploration::FrontierExploration(ros::NodeHandle private_nh_)
    : m_nh(),
    m_maxRange(10.0),
    m_filterGroundPlane(false),
    m_colorFactor(0.8),
    m_pointcloudMinX(-std::numeric_limits<double>::max()),
    m_pointcloudMaxX(std::numeric_limits<double>::max()),
    m_pointcloudMinY(-std::numeric_limits<double>::max()),
    m_pointcloudMaxY(std::numeric_limits<double>::max()),
    m_pointcloudMinZ(-std::numeric_limits<double>::max()),
    m_pointcloudMaxZ(std::numeric_limits<double>::max()),
    m_occupancyMinZ(-std::numeric_limits<double>::max()),
    m_occupancyMaxZ(std::numeric_limits<double>::max()),
    numberOfUnknowns(18),
    m_worldFrameId("world"),
    cloud_in("/velodyne_points"),
    // cloud_in("/m100/vi_sensor/camera_depth/depth/points"),
    //cloud_in("/m100/velodyne_points"),
    m_res(0.5),
    m_treeDepth(0),
    m_maxTreeDepth(0),
    xFiltering(false),
    yFiltering(false),
    zFiltering(false),
    publishTheMap(true),
    m_distance_deletion(8.0)
   {



    double probHit, probMiss, thresMin, thresMax;

    ros::NodeHandle private_nh = private_nh_;
    //Set variables
    private_nh.param("frame_id", m_worldFrameId, m_worldFrameId);
    private_nh.param("publish_the_map", publishTheMap, publishTheMap);
    private_nh.param("resolution", m_res, m_res);
    private_nh.param("sensor_model/hit", probHit, 0.7);
    private_nh.param("sensor_model/miss", probMiss, 0.4);
    private_nh.param("sensor_model/min", thresMin, 0.12);
    private_nh.param("sensor_model/max", thresMax, 0.97);
    private_nh.param("sensor_model/max_range", m_maxRange, m_maxRange);
    private_nh.param("occupancy_min_z", m_occupancyMinZ,m_occupancyMinZ);
    private_nh.param("occupancy_max_z", m_occupancyMaxZ,m_occupancyMaxZ);
    private_nh.param("color_factor", m_colorFactor, m_colorFactor);
    private_nh.param("cloud_in", cloud_in, cloud_in);
    private_nh.param("distance", m_distance_deletion, m_distance_deletion);

    private_nh.param("number_of_unknowns", numberOfUnknowns, numberOfUnknowns);

    //Filtering
    private_nh.param("xFiltering", xFiltering, xFiltering);
    private_nh.param("yFiltering", yFiltering, yFiltering);
    private_nh.param("zFiltering", zFiltering, zFiltering);

    //Setup octree
    m_octree = new OcTreeT(m_res);
    m_octree->setProbHit(probHit);
    m_octree->setProbMiss(probMiss);
    m_octree->setClampingThresMin(thresMin);
    m_octree->setClampingThresMax(thresMax);
    m_octree->enableChangeDetection(true);
    m_treeDepth = m_octree->getTreeDepth();
    m_maxTreeDepth = m_treeDepth;

    double r, g, b, a;
    private_nh.param("color/r", r, 0.0);
    private_nh.param("color/g", g, 0.0);
    private_nh.param("color/b", b, 1.0);
    private_nh.param("color/a", a, 1.0);
    m_color.r = r;
    m_color.g = g;
    m_color.b = b;
    m_color.a = a;

    private_nh.param("color_free/r", r, 0.0);
    private_nh.param("color_free/g", g, 1.0);
    private_nh.param("color_free/b", b, 0.0);
    private_nh.param("color_free/a", a, 1.0);
    m_colorFree.r = r;
    m_colorFree.g = g;
    m_colorFree.b = b;
    m_colorFree.a = a;

    //Register to the subscriber topic and register it to the callback function
    m_pointCloudSubscriber = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, cloud_in, 5);
    m_tfPointCloudSubscriber = new tf::MessageFilter<sensor_msgs::PointCloud2> (*m_pointCloudSubscriber, m_tfListener, m_worldFrameId, 5);
    m_tfPointCloudSubscriber->registerCallback(boost::bind(&FrontierExploration::insertPointCloudCallback, this, _1));

    //Register publishers
    m_frontierPub = m_nh.advertise<visualization_msgs::MarkerArray>("/frontier_nodes", 1, true);
    m_frontierSelectionPub = m_nh.advertise<ref::Frontiers>("frontiers", 1, true);
    m_occupiedPub = m_nh.advertise<visualization_msgs::MarkerArray>("/occupied_nodes", 1, true);
};

void FrontierExploration::insertPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud) {
    PCLPointCloud pc;
    pcl::fromROSMsg(*cloud, pc);

    tf::StampedTransform sensorToWorldTf;
    try {
        m_tfListener.lookupTransform(m_worldFrameId, cloud->header.frame_id, cloud->header.stamp, sensorToWorldTf);
    } catch(tf::TransformException& ex) {
        ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
        return;
    }

    Eigen::Matrix4f sensorToWorld;
    pcl_ros::transformAsMatrix(sensorToWorldTf, sensorToWorld);

    /***********************************
     ******** Filtering ****************
     **********************************/

    pcl::PassThrough<PCLPoint> pass_x;
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(m_pointcloudMinX, m_pointcloudMaxX);

    pcl::PassThrough<PCLPoint> pass_y;
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(m_pointcloudMinY, m_pointcloudMaxY);

    pcl::PassThrough<PCLPoint> pass_z;
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(m_pointcloudMinZ, m_pointcloudMaxZ);

    PCLPointCloud pc_ground;
    PCLPointCloud pc_nonground;

    pcl::transformPointCloud(pc, pc, sensorToWorld);
    // just filter height range:
    pass_x.setInputCloud(pc.makeShared());
    pass_x.filter(pc);
    pass_y.setInputCloud(pc.makeShared());
    pass_y.filter(pc);
    pass_z.setInputCloud(pc.makeShared());
    pass_z.filter(pc);

    pc_nonground = pc;
    pc_ground.header = pc.header;
    pc_nonground.header = pc.header;

    insertPointCloudScan(sensorToWorldTf.getOrigin(), pc_ground, pc_nonground);
    publishFrontiers(sensorToWorldTf.getOrigin(), cloud->header.stamp);
    if (publishTheMap) {
        publishMap(cloud->header.stamp);
    }

}

/*
This function distingguish between frontiers and what is not frontiers.
If a point outside our manually set radius is hit, it is marked a frontier on the same distance as the radius.
Use Rviz for visualization.

** NOTICE **
Frontier cells is marked as occupied in the octree.
*/
void FrontierExploration::insertPointCloudScan(const tf::Point& sensorOriginTf, const PCLPointCloud& ground, const PCLPointCloud& nonground) {
    point3d sensorOrigin = octomap::pointTfToOctomap(sensorOriginTf);
    if (!m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMin) || !m_octree->coordToKeyChecked(sensorOrigin, m_updateBBXMax)) {
        ROS_ERROR_STREAM("Could not generate Key for origin "<<sensorOrigin);
    }
    KeySet free_cells, occupied_cells;

    for (PCLPointCloud::const_iterator it = nonground.begin(); it != nonground.end(); ++it) {
        point3d point(it->x, it->y, it->z);
        //Check if point is within our radius
        if ((m_maxRange < 0.0) || ((point - sensorOrigin).norm() <= m_maxRange) ) {

            // Add all point to the point as free cells
            if (m_octree->computeRayKeys(sensorOrigin, point, m_keyRay)){
                free_cells.insert(m_keyRay.begin(), m_keyRay.end());
            }

            // Add the point as occupied cell
            octomap::OcTreeKey key;
            if (m_octree->coordToKeyChecked(point, key)){
                occupied_cells.insert(key);
                updateMinKey(key, m_updateBBXMin);
                updateMaxKey(key, m_updateBBXMax);
            }
        } else {
            point3d new_end = sensorOrigin + (point - sensorOrigin).normalized() * m_maxRange;
            if (m_octree->computeRayKeys(sensorOrigin, new_end, m_keyRay)){
                free_cells.insert(m_keyRay.begin(), m_keyRay.end());
                octomap::OcTreeKey endKey;
                if (m_octree->coordToKeyChecked(new_end, endKey)){
                    free_cells.insert(endKey);
                    updateMinKey(endKey, m_updateBBXMin);
                    updateMaxKey(endKey, m_updateBBXMax);
                } else{
                ROS_ERROR_STREAM("Could not generate Key for endpoint "<<new_end);
                }


            }
        }
    }



    // mark free cells only if not seen occupied in this cloud
    for(KeySet::iterator it = free_cells.begin(), end=free_cells.end(); it!= end; ++it){
        if (occupied_cells.find(*it) == occupied_cells.end()){
            m_octree->updateNode(*it, false);
        }
    }

    // now mark all occupied cells:
    for (KeySet::iterator it = occupied_cells.begin(), end=occupied_cells.end(); it!= end; it++) {
        m_octree->updateNode(*it, true);
    }

    octomap::point3d minPt, maxPt;

    minPt = m_octree->keyToCoord(m_updateBBXMin);
    maxPt = m_octree->keyToCoord(m_updateBBXMax);
}

void FrontierExploration::publishMap(const ros::Time& rostime) {
    visualization_msgs::MarkerArray occupiedNodesVis;
    occupiedNodesVis.markers.resize(m_treeDepth+1);
     for (OcTreeT::iterator it = m_octree->begin(m_maxTreeDepth), end = m_octree->end(); it != end; ++it) {
         if (m_octree->isNodeOccupied(*it)){
            double z = it.getZ();
            // if(z < 0 || z > 2.8) {
                // continue;
            // }
            double half_size = it.getSize() / 2.0;

            if (z + half_size > m_occupancyMinZ && z - half_size < m_occupancyMaxZ) {
                double size = it.getSize();
                double x = it.getX();
                double y = it.getY();

                unsigned idx = it.getDepth();
                geometry_msgs::Point cubeCenter;
                cubeCenter.x = x;
                cubeCenter.y = y;
                cubeCenter.z = z;
                // if(cubeCenter.z > 2.5) {

                // } else {

                occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
                    double minX, minY, minZ, maxX, maxY, maxZ;
                    m_octree->getMetricMin(minX, minY, minZ);
                    m_octree->getMetricMax(maxX, maxY, maxZ);

                    double h = (1.0 - std::min(std::max((cubeCenter.z-minZ)/ (maxZ - minZ), 0.0), 1.0)) *m_colorFactor;
                    occupiedNodesVis.markers[idx].colors.push_back(heightMapColor(h));
            // }
            }
        }
    }
    int hej = 0;
    for (unsigned i= 0; i < occupiedNodesVis.markers.size(); ++i){
      //  ROS_INFO("%f", hej);
      double size = m_octree->getNodeSize(i);
      occupiedNodesVis.markers[i].header.frame_id = m_worldFrameId;
      occupiedNodesVis.markers[i].header.stamp = rostime;
      occupiedNodesVis.markers[i].ns = "map";
      occupiedNodesVis.markers[i].id = i;
      occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
      occupiedNodesVis.markers[i].scale.x = (m_res);
      occupiedNodesVis.markers[i].scale.y = (m_res);
      occupiedNodesVis.markers[i].scale.z = (m_res);
        hej += occupiedNodesVis.markers[i].points.size();
      if (occupiedNodesVis.markers[i].points.size() > 0)
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
      else
        occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
    }
    // ROS_INFO("POINTS: %i", hej/180);
    m_occupiedPub.publish(occupiedNodesVis);
}


void FrontierExploration::publishFrontiers(const tf::Point& sensorOriginTf, const ros::Time& rostime) {
    point3d sensorOrigin = octomap::pointTfToOctomap(sensorOriginTf);

    visualization_msgs::MarkerArray frontierVis;
    ref::Frontiers frontierPoints;

    frontierVis.markers.resize(m_octree->numChangesDetected()+frontierPointList.size()+1);

    //Go through the old list of Frontier keys and check if they are still frontiers
    for (octomap::point3d_list::iterator it = frontierPointList.begin(), end = frontierPointList.end(); it != end; ++it) {

        float x = it->x();
        float y = it->y();
        float z = it->z();
        if (!FrontierExploration::isFrontier(x, y, z, sensorOrigin)) {
            it = frontierPointList.erase(it);
        }
    }

    //Go through all changed keys, add them to the frontier list if they are concidered to be a frontier.
    for(KeyBoolMap::const_iterator it = m_octree->changedKeysBegin(), end = m_octree->changedKeysEnd(); it != end; ++it) {
        if(it->second == true){
            octomap::OcTreeNode* node = m_octree->search(it->first,0);
            if(node != NULL) {
                if (!m_octree->isNodeOccupied(node)) {
                    point3d point = m_octree->keyToCoord(it->first);
                    float x = point.x();
                    float y = point.y();
                    float z = point.z();
                    octomap::OcTreeNode* me;
                    me = m_octree->search(x,y,z);

                    point3d pt;
                    pt.x() = point.x();
                    pt.y() = point.y();
                    pt.z() = point.z();
                    
                    // if (pt.z() > 0.8 && pt.z() < 2.2) {
                         
                        
                        if (FrontierExploration::isFrontier(x, y, z, sensorOrigin) && !(m_octree->isNodeOccupied(me))) {
                        frontierPointList.push_back(pt);
                        }

                // }
                    
                }
            }
        }
    }

    //For visual and topic publishing
    int idx = 0;
    std_msgs::ColorRGBA color;
    color.a = 0.5;
    color.r = 1; color.g = 1; color.b = 0;
    for (point3d point : frontierPointList) {
        geometry_msgs::Point cubeCenter;
        cubeCenter.x = point.x();
        cubeCenter.y = point.y();
        cubeCenter.z = point.z();
        frontierVis.markers[idx].color.a =  1.0;
        frontierVis.markers[idx].points.push_back(cubeCenter);
        frontierVis.markers[idx].colors.push_back(color);
        frontierPoints.points.push_back(cubeCenter);
        idx++;
    }

    m_octree->resetChangeDetection();

    if (frontierVis.markers.size() > 0){
        for (unsigned i= 0; i < frontierVis.markers.size(); ++i){
            //double size = m_octree->getNodeSize(i);
            frontierVis.markers[i].header.frame_id = m_worldFrameId;
            frontierVis.markers[i].header.stamp = rostime;
            frontierVis.markers[i].ns = i;
            frontierVis.markers[i].id = i;
            frontierVis.markers[i].type = visualization_msgs::Marker::SPHERE_LIST;
            frontierVis.markers[i].scale.x = (m_res);
            frontierVis.markers[i].scale.y = (m_res);
            frontierVis.markers[i].scale.z = (m_res);
            if (frontierVis.markers[i].points.size() > 0)
                frontierVis.markers[i].action = visualization_msgs::Marker::ADD;
    }
    m_frontierPub.publish(frontierVis);
    m_frontierSelectionPub.publish(frontierPoints);
    }

}


bool FrontierExploration::isFrontier(float x, float y, float z, point3d sensorOrigin) {

    octomap::OcTreeNode* me;
    octomap::OcTreeNode* nodeX;
    octomap::OcTreeNode* nodeY;
    octomap::OcTreeNode* nodeZ;
    octomap::OcTreeNode* nodeNegativeX;
    octomap::OcTreeNode* nodeNegativeY;
    octomap::OcTreeNode* nodeNegativeZ;
    point3d frontier(x,y,z);
    me = m_octree->search(x,y,z);
    if(m_octree->isNodeOccupied(me))
    {
        return false;
    }
    else {
        //North
        octomap::OcTreeNode* N = m_octree->search(x+(m_res ),y,z,0);
        octomap::OcTreeNode* NW = m_octree->search(x+(m_res ),y+(m_res ),z,0);
        octomap::OcTreeNode* NE = m_octree->search(x+(m_res ),y-(m_res ), z,0);
        octomap::OcTreeNode* NT = m_octree->search(x+(m_res ),y,z+(m_res ),0);
        octomap::OcTreeNode* NTW = m_octree->search(x+(m_res ),y+(m_res ),z+(m_res ),0);
        octomap::OcTreeNode* NTE = m_octree->search(x+(m_res ),y-(m_res ), z+(m_res ),0);
        octomap::OcTreeNode* NB = m_octree->search(x+(m_res ),y,z-(m_res ),0);
        octomap::OcTreeNode* NBW = m_octree->search(x+(m_res ),y+(m_res ),z-(m_res ),0);
        octomap::OcTreeNode* NBE = m_octree->search(x+(m_res ),y-(m_res ), z-(m_res ),0);
        //Mid
        octomap::OcTreeNode* W = m_octree->search(x,y+(m_res ),z,0);
        octomap::OcTreeNode* E = m_octree->search(x,y-(m_res ), z,0);
        octomap::OcTreeNode* T = m_octree->search(x,y,z+(m_res ),0);
        octomap::OcTreeNode* TW = m_octree->search(x,y+(m_res ),z+(m_res ),0);
        octomap::OcTreeNode* TE = m_octree->search(x,y-(m_res ), z+(m_res ),0);
        octomap::OcTreeNode* B = m_octree->search(x,y,z-(m_res ),0);
        octomap::OcTreeNode* BW = m_octree->search(x,y+(m_res ),z-(m_res ),0);
        octomap::OcTreeNode* BE = m_octree->search(x,y-(m_res ), z-(m_res ),0);
        //South
        octomap::OcTreeNode* S = m_octree->search(x-(m_res ),y,z,0);
        octomap::OcTreeNode* SW = m_octree->search(x-(m_res ),y+(m_res ),z,0);
        octomap::OcTreeNode* SE = m_octree->search(x-(m_res ),y-(m_res ), z,0);
        octomap::OcTreeNode* ST = m_octree->search(x-(m_res ),y,z+(m_res ),0);
        octomap::OcTreeNode* STW = m_octree->search(x-(m_res ),y+(m_res ),z+(m_res ),0);
        octomap::OcTreeNode* STE = m_octree->search(x-(m_res ),y-(m_res ), z+(m_res ),0);
        octomap::OcTreeNode* SB = m_octree->search(x-(m_res ),y,z-(m_res ),0);
        octomap::OcTreeNode* SBW = m_octree->search(x-(m_res ),y+(m_res ),z-(m_res ),0);
        octomap::OcTreeNode* SBE = m_octree->search(x-(m_res ),y-(m_res ), z-(m_res ),0);

        int points = 0;

        if (!N) {
            points++;
        } else {
            if(m_octree->isNodeOccupied(N)) {
                return false;
            }
        }
        if (!NW) {
            points++;
        }
        else {
            if(m_octree->isNodeOccupied(NW)) {
                return false;
            }
        }
        if (!NE) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(NE)) {
                return false;
            }
        }
        if (!NT) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(NT)) {
                return false;
            }
        }
        if (!NTW) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(NTW)) {
                return false;
            }
        }
        if (!NTE) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(NTE)) {
                return false;
            }
        }
        if (!NB) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(NB)) {
                return false;
            }
        }
        if (!NBW) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(NBW)) {
                return false;
            }
        }
        if (!NBE) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(NBE)) {
                return false;
            }
        }

        if (!W) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(W)) {
                return false;
            }
        }
        if (!E) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(E)) {
                return false;
            }
        }
        if (!T) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(T)) {
                return false;
            }
        }
        if (!TW) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(TW)) {
                return false;
            }
        }
        if (!TE) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(TE)) {
                return false;
            }
        }
        if (!B) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(B)) {
                return false;
            }
        }
        if (!BW) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(BW)) {
                return false;
            }
        }
        if (!BE) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(BE)) {
                return false;
            }
        }


        if (!S) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(S)) {
                return false;
            }
        }
        if (!SW) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(SW)) {
                return false;
            }
        }
        if (!SE) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(SE)) {
                return false;
            }
        }
        if (!ST) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(ST)) {
                return false;
            }
        }
        if (!STW) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(STW)) {
                return false;
            }
        }
        if (!STE) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(STE)) {
                return false;
            }
        }
        if (!SB) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(SB)) {
                return false;
            }
        }
        if (!SBW) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(SBW)) {
                return false;
            }
        }
        if (!SBE) {
            points++;
        }else {
            if(m_octree->isNodeOccupied(SBE)) {
                return false;
            }
        }

    float distance = (sensorOrigin - frontier).norm();


    if (points >= numberOfUnknowns && distance > m_distance_deletion ) {
        return true;
    } else {
        return false;
    }
}


}

std_msgs::ColorRGBA FrontierExploration::heightMapColor(double h) {

  std_msgs::ColorRGBA color;
  color.a = 1.0;
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
      color.r = v; color.g = n; color.b = m;
      break;
    case 1:
      color.r = n; color.g = v; color.b = m;
      break;
    case 2:
      color.r = m; color.g = v; color.b = n;
      break;
    case 3:
      color.r = m; color.g = n; color.b = v;
      break;
    case 4:
      color.r = n; color.g = m; color.b = v;
      break;
    case 5:
      color.r = v; color.g = m; color.b = n;
      break;
    default:
      color.r = 1; color.g = 0.5; color.b = 0.5;
      break;
  }

  return color;
}
