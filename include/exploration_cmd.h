#ifndef EXPLORATION_CMD_H
#define EXPLORATION_CMD_H

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <mav_msgs/Actuators.h>
#include <mav_msgs/AttitudeThrust.h>
#include <mav_msgs/eigen_mav_msgs.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <octomap/octomap.h>
#include <mav_planning_msgs/PlannerService.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <ewok/polynomial_3d_optimization.h>
#include <ewok/uniform_bspline_3d_optimization.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <tf_conversions/tf_eigen.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>

#include <Eigen/Core>
#include <Eigen/Eigen>

#include <exploration/Frontier.h>

class ExplorationCMD {
    public:
        ExplorationCMD(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));
        ros::NodeHandle m_nh;

        typedef pcl::PointCloud<pcl::PointXYZ> PCLPointCloud; 
        typedef pcl::PointXYZ PCLPoint;

        

        message_filters::Subscriber<std_msgs::String>* commandSub;
        message_filters::Subscriber<std_msgs::Empty>* updateSub;
        message_filters::Subscriber<exploration::Frontier>* pointSub;
        message_filters::Subscriber<nav_msgs::Odometry>* m_poseSubscriber;
        message_filters::Subscriber<sensor_msgs::PointCloud2>* pointCloudSubscriber; 

        ros::ServiceClient pathService;
        ros::ServiceClient publishPathService;
        ros::Timer timer;
        ros::Timer globalFrontierTimer;

        tf::TransformListener m_tfListener;
        tf::TransformListener * listener;
        tf::MessageFilter<sensor_msgs::PointCloud2>* m_tfPointCloudSubscriber;

        
        ewok::PolynomialTrajectory3D<10>::Ptr traj;
        ewok::EuclideanDistanceRingBuffer<6>::Ptr edrb;
        ewok::UniformBSpline3DOptimization<6>::Ptr spline_optimization;
        
        nav_msgs::Path path;
        ros::Publisher m_frontierVisPub;
        ros::Publisher nextPointPub;
        ros::Publisher nextDirectPointPub;
        ros::Publisher traj_marker_pub;
        ros::Publisher current_traj_pub;
        ros::Publisher dist_marker_pub;
        ros::Publisher occ_marker_pub;
        ros::Publisher free_marker_pub;
        ros::Publisher trajectory_pub;
        ros::Publisher point_pub;
        ros::Publisher path_pub;

        virtual void commandHandler(const std_msgs::String::ConstPtr& message);
        virtual void pointHandler(const exploration::Frontier::ConstPtr& point);
        virtual void updateCurrentPosition(const nav_msgs::Odometry::ConstPtr& pose);
        virtual void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pc);
        virtual void sendCommandCallback(const ros::TimerEvent& event);

        bool explore = false;
        bool firstTime = true;
        octomap::point3d frontierPoint;
        octomap::point3d my_current_pos;
        nav_msgs::Odometry m_currentPose;
        geometry_msgs::Point nextPoint;
        // octomap::point3d first_opt;
        octomap::point3d mine;

        std::string cloud;
        std::string publishPointTopic;
        std::string odom;
        
        double max_velocity; 
        double max_acceleration;
        double distance_threshold;
        double dt;
        int field;

        int num_opt_points;
        double last_yaw;

        Eigen::Vector4d limits;
        visualization_msgs::MarkerArray frontierVis;
        float resolution;
        bool initialized = false;
        double distanceBeforeSelectingNew;
        double stopTimeGlobalFrontier;
        ros::Time startTime;
        ros::Time endTime;
        bool init_explore = true;

};
#endif