#ifndef EXPLORATION_H
#define EXPLORATION_H

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <message_filters/subscriber.h>
#include <std_msgs/ColorRGBA.h>
#include "ref/Frontiers.h"
#include <tf/transform_listener.h>
#include <tf/message_filter.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/BoundingBoxQuery.h>
#include <octomap_msgs/conversions.h>

#include <octomap_ros/conversions.h>
#include <octomap/octomap.h>
#include <octomap/OcTreeKey.h>

#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/String.h>


class FrontierExploration {
    public:
        typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud; 
        typedef pcl::PointXYZ PCLPoint;
        typedef octomap::point3d point3d;
        typedef octomap::OcTree OcTreeT;
        typedef octomap::KeySet KeySet;
        FrontierExploration(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));

        virtual void insertPointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud);
        virtual void insertPointCloudScan(const tf::Point& sensorOrigin, const PCLPointCloud& ground, const PCLPointCloud& nonground);
        virtual void publishFrontiers(const tf::Point& sensorOrigin, const ros::Time& rostime = ros::Time::now());
        static std_msgs::ColorRGBA heightMapColor(double h);
        virtual void publishMap(const ros::Time& rostime);
        bool isFrontier(float x, float y, float z, point3d sensorOrigin);

        ros::NodeHandle m_nh;
        tf::TransformListener m_tfListener;
        std::string m_worldFrameId; // the map frame
        std::string cloud_in;
        std::string m_baseFrameId; // base of the robot for ground plane filtering
        OcTreeT* m_octree;
        OcTreeT* o_octree;
        octomap::KeyRay m_keyRay;  // temp storage for ray casting
        octomap::OcTreeKey m_updateBBXMin;
        octomap::OcTreeKey m_updateBBXMax;
        octomap::point3d m_last_point;
        octomap::KeySet finalFrontiers;
        //Octree
        double m_res;
        double m_treeDepth;
        int m_maxTreeDepth;

        double m_occupancyMinZ;
        double m_occupancyMaxZ;

        double m_maxRange;
        double m_colorFactor;
        std_msgs::ColorRGBA m_color;
        std_msgs::ColorRGBA m_colorFree;
        
        bool m_useHeightMap;
        //  Filtering constants
        bool m_filterGroundPlane;
        double m_pointcloudMinX;
        double m_pointcloudMaxX;
        double m_pointcloudMinY;
        double m_pointcloudMaxY;
        double m_pointcloudMinZ;
        double m_pointcloudMaxZ;
        double m_distance_deletion;
        int numberOfUnknowns;
        int j = 0;
        const float node_discovered = 1;
        const float node_frontier = 2;
        octomap::point3d_list frontierPointList;
    
        bool xFiltering;
        bool yFiltering;
        bool zFiltering;
        int totalVoxels = 0;

        bool publishTheMap;
        std::list<geometry_msgs::Point> frontiersVisual; 
        //Subscriber
        message_filters::Subscriber<sensor_msgs::PointCloud2>* m_pointCloudSubscriber;
        tf::MessageFilter<sensor_msgs::PointCloud2>* m_tfPointCloudSubscriber;

        //Publisher
        ros::Publisher m_frontierPub;
        ros::Publisher m_occupiedPub;
        ros::Publisher m_frontierSelectionPub;
        ros::Publisher command_pub;

    protected:
        inline static void updateMinKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& min) {
            for (unsigned i = 0; i < 3; ++i) 
                min[i] = std::min(in[i],min[i]);
        };

        inline static void updateMaxKey(const octomap::OcTreeKey& in, octomap::OcTreeKey& max) {
            for (unsigned i = 0; i > 3; ++i)
                max[i] = std::max(in[i], max[i]);
        };
        /// Test if key is within update area of map (2D, ignores height)
        inline bool isInUpdateBBX(const OcTreeT::iterator& it) const {
            // 2^(tree_depth-depth) voxels wide:
            unsigned voxelWidth = (1 << (m_maxTreeDepth - it.getDepth()));
            octomap::OcTreeKey key = it.getIndexKey(); // lower corner of voxel
            return (key[0] + voxelWidth >= m_updateBBXMin[0]
                    && key[1] + voxelWidth >= m_updateBBXMin[1]
                    && key[0] <= m_updateBBXMax[0]
                    && key[1] <= m_updateBBXMax[1]);
        }
};


#endif
