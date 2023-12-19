#ifndef SELECTION_H
#define SELECTION_H

#include <ros/ros.h>
#include <math.h>
#include <ros/console.h>
#include <message_filters/subscriber.h>
#include <mav_msgs/default_topics.h>
#include <ref/Frontiers.h>
#include <ref/Frontier.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <octomap/octomap.h>
#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <visualization_msgs/MarkerArray.h>


class FrontierSelection {
    public:
        FrontierSelection(ros::NodeHandle private_nh_ = ros::NodeHandle("~"));
        ros::NodeHandle m_nh;
        octomath::Vector3 m_directionVector;
        nav_msgs::Odometry m_currentPose;
        nav_msgs::Odometry m_prevPose; //Holds the point where the direction changed.
        ref::Frontier aka;
        ref::Frontier aka_g;
        octomap::point3d frontierPoint;
	geometry_msgs::Point minFrontier;
        geometry_msgs::Point minPoint;
        geometry_msgs::Point maxPoint;
        geometry_msgs::Point selectedfrontier;
        geometry_msgs::Point nextfrontier;
        geometry_msgs::Point frontier;
        geometry_msgs::Point myPoint;
        geometry_msgs::Point localf;
        geometry_msgs::PoseStamped localfpose;
        geometry_msgs::PoseStamped globalfpose;
        geometry_msgs::Point globalf;
        geometry_msgs::Point my;
        geometry_msgs::Point localfrontier;
        // geometry_msgs::Point localfrontier;
        octomap::point3d lastPoint;
        octomap::point3d oldPoint;
        octomap::point3d newPoint;
        octomap::point3d myp;
        octomap::point3d origin;
	octomap::point3d starting;
        octomap::point3d lastpublished;
        octomath::Vector3 heading;
        octomap::point3d frontierHeading;
        octomap::point3d dronepose;
        octomath::Vector3 frontierVector;
        geometry_msgs::Point pub;
        octomap::point3d pubpt;
        float reach_dis;

        Eigen::Quaterniond attitude;
        Eigen::Vector3d euler;
        double yaw_angle;
        double new_yaw_angle;
        double head;
        double new_head;
        // yaw_angle = euler.z() * (180/3.14159);
        

        //Functions
        virtual void handleInput(const ref::Frontiers::ConstPtr& markers);
        virtual void updateCurrentPosition(const nav_msgs::Odometry::ConstPtr& pos);
        // virtual float calculateAngle(octomath::Vector3 vector1, octomath::Vector3 vector2);
        virtual void publishFrontier(visualization_msgs::MarkerArray frontierVis);
        // virtual float weightFrontier(geometry_msgs::Point frontier);
        // virtual void updateDirection(octomap::point3d origin, octomap::point3d frontier );
        virtual geometry_msgs::Point weightGlobalFrontier(std::list<geometry_msgs::Point> frontiers);
        virtual geometry_msgs::Point weightLocalFrontier(std::list<geometry_msgs::Point> frontiers);
        virtual double calculateDistance(octomap::point3d origin, octomap::point3d point);
        //Subscribers
        message_filters::Subscriber<ref::Frontiers>* m_frontierSubscriber;
        message_filters::Subscriber<nav_msgs::Odometry>* m_poseSubscriber; //Subscribes to the pose of the quadcopter


        //Publishers
        ros::Publisher m_frontierVisPub;
        ros::Publisher m_frontierPub;
        ros::Publisher m_frontierPubPose;
        ros::Publisher m_pub_frontier;
        ros::Publisher stopPub;
        //Params
        float fovAngle;
        float minDistance;
        float minScore = 9999999;
        float score;
        float heightDifference;
        float minHeightDiff;
        float minAngle;
        float maxScore;
        float myScore;
        float midcost;



        double angle;
        double distance;
        double distanceConstant;
        double angleConstant;
        double heightConstant;
        double angleValue;
        double distanceValue;
        double m_res;
        double distant;
        double psi;
        double localRange;
        double dis;
        int sensorRange;
        int rangeToLocalFrontiers;

        std::string m_worldFrameId; // the map frame
        std::string m_frontierTopic;

        bool firstTime = true;
        bool start_flag = true;
        bool move_flag;
        bool next_pub;
        
        // bool move_flag;
};


#endif
