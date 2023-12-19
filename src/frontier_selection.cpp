#include "frontier_selection.h"
#include <cmath>

using namespace std;

FrontierSelection::FrontierSelection(ros::NodeHandle private_nh_)
    : m_nh(),
    heightConstant(10),
    angleConstant(2),
    distanceConstant(10),
    fovAngle(40),
    m_worldFrameId("world"),
    m_frontierTopic("frontiers"),
    m_res(0.25),
    sensorRange(10),
    rangeToLocalFrontiers(20)

     {
        ros::NodeHandle private_nh = private_nh_;

        private_nh.param("height_constant", heightConstant, heightConstant);
        private_nh.param("angle_constant", angleConstant,angleConstant);
        private_nh.param("distance_constant", distanceConstant, distanceConstant);
        private_nh.param("fov_angle", fovAngle, fovAngle);
        private_nh.param("frame_id", m_worldFrameId, m_worldFrameId);
        private_nh.param("frontier_topic", m_frontierTopic, m_frontierTopic);
        private_nh.param("resolution", m_res, m_res);
        private_nh.param("sensor_range", sensorRange, sensorRange);
        private_nh.param("local_range", localRange, localRange);
        private_nh.param("range_to_local_frontiers", rangeToLocalFrontiers, rangeToLocalFrontiers);


        m_frontierVisPub = m_nh.advertise<visualization_msgs::MarkerArray>("/local_frontiers", 1, true);
        m_frontierPub = m_nh.advertise<geometry_msgs::Point>("/frontier/point", 1, true);
        m_pub_frontier = m_nh.advertise<ref::Frontier>("/cmd_frontier", 1, true);
        m_frontierPubPose = m_nh.advertise<geometry_msgs::PoseStamped>("/frontier/pose", 1, true);
        stopPub = m_nh.advertise<std_msgs::String>("/command", 1, true);


        //Register to the subscribers to topic
        m_frontierSubscriber = new message_filters::Subscriber<ref::Frontiers> (m_nh, m_frontierTopic, 5);
        m_poseSubscriber = new message_filters::Subscriber<nav_msgs::Odometry> (m_nh, "/hummingbird/ground_truth/odometry", 5);

        //Register subscribers to callback functions
        m_frontierSubscriber->registerCallback(boost::bind(&FrontierSelection::handleInput, this, _1));
        m_poseSubscriber->registerCallback(boost::bind(&FrontierSelection::updateCurrentPosition, this, _1));


    };


    Eigen::Vector3d quaternion_to_rpy(Eigen::Quaterniond q)
    {
        const double roll = atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                            1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
        const double pitch = asin(2.0 * (q.w() * q.y() - q.z() * q.x()));
        const double yaw = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                           1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));

        return Eigen::Vector3d{roll, pitch, yaw};
    }


    void FrontierSelection::handleInput(const ref::Frontiers::ConstPtr& markers) {
        std::list<geometry_msgs::Point> local;
        std::list<geometry_msgs::Point> global;
        std::list<geometry_msgs::Point> seg1;
        std::list<geometry_msgs::Point> seg2;
        std::list<geometry_msgs::Point> seg3;
        std::list<geometry_msgs::Point> seg4;
        std::list<geometry_msgs::Point> seg5;
        std::list<geometry_msgs::Point> seg6;
        std::list<geometry_msgs::Point> seg7;
        std::list<geometry_msgs::Point> seg8;
        std::list<geometry_msgs::Point> myseg;
        
        visualization_msgs::MarkerArray frontierVis;
        int i = 0;

        //If there  is no new frontiers coming in we stop at the position we're at.
        if (markers->points.empty()) {
            frontierVis.markers.resize(1);
            minFrontier.x = m_currentPose.pose.pose.position.x;
            minFrontier.y = m_currentPose.pose.pose.position.y;
            minFrontier.z = m_currentPose.pose.pose.position.z;
            nextfrontier.x = minFrontier.x;
            nextfrontier.y = minFrontier.y;
            nextfrontier.z = minFrontier.z;
            
            m_frontierPub.publish(nextfrontier);

        } else {
        //Else we go through each frontier
            frontierVis.markers.resize(markers->points.size());

            minAngle = 360.0;
            minHeightDiff = 99999;
            for (geometry_msgs::Point frontier : markers->points) {

                //Convert msg to octomap points
                origin = octomap::point3d(m_currentPose.pose.pose.position.x, m_currentPose.pose.pose.position.y, m_currentPose.pose.pose.position.z);
                frontierPoint = octomap::point3d(frontier.x, frontier.y, frontier.z);

                frontierHeading = octomap::point3d(frontier.x, frontier.y, 0);
                dronepose = octomap::point3d(m_currentPose.pose.pose.position.x, m_currentPose.pose.pose.position.y, 0);

                //Calculate vector
                frontierVector = octomath::Vector3(frontierHeading-dronepose).normalize();

                heightDifference = abs(frontier.z - origin.z());
                distance = calculateDistance(origin, frontierPoint);

                head = (atan2((frontierPoint.y() - dronepose.y()), (frontierPoint.x() - dronepose.x())) * (180/3.14159));
                
                angle = head - yaw_angle;

                //Check if the frontier is in our field of view
                // if (((angle < fovAngle) && (angle > -fovAngle)) && (distance < 18) && heightDifference < 0.6) {

                if (((angle < fovAngle) && (angle > -fovAngle)) && distance < 15) {
                    
                    local.push_back(frontier);
                    local.resize(local.size());

                    if ((angle < -30) && (angle > -40)) {
                        
                        seg1.push_back(frontier);
                        

                    }
                    if ((angle < -20) && (angle > -30)) {
                        
                        seg2.push_back(frontier);
                        

                    }
                    if ((angle < -10) && (angle > -20)) {
                        
                        seg3.push_back(frontier);
                        

                    }
                    if ((angle < 0) && (angle > -10)) {
                        
                        seg4.push_back(frontier);
                        

                    }
                    if ((angle < 10) && (angle > 0)) {
                        
                        seg5.push_back(frontier);
                        

                    }
                    if ((angle < 20) && (angle > 10)) {
                        
                        seg6.push_back(frontier);
                        

                    }
                    if ((angle < 30) && (angle > 20)) {
                        
                        seg7.push_back(frontier);
                        

                    }
                    if ((angle < 40) && (angle > 30)) {
                        
                        seg8.push_back(frontier);
                        

                    }

                    if (seg1.size() > myseg.size()) {
                        myseg = seg1;
                    }
                    if (seg2.size() > myseg.size()) {
                        myseg = seg2;
                    }
                    if (seg3.size() > myseg.size()) {
                        myseg = seg3;
                    }
                    if (seg4.size() > myseg.size()) {
                        myseg = seg4;
                    }
                    if (seg5.size() > myseg.size()) {
                        myseg = seg5;
                    }
                    if (seg6.size() > myseg.size()) {
                        myseg = seg6;
                    }
                    if (seg7.size() > myseg.size()) {
                        myseg = seg7;
                    }
                    if (seg8.size() > myseg.size()) {
                        myseg = seg8;
                    }
                    // cout << "myseg_size: " << myseg.size() << std::endl;
                            
                    geometry_msgs::Point frontierlocal = FrontierSelection::weightLocalFrontier(myseg);

                    localf.x = frontierlocal.x;
                    localf.y = frontierlocal.y;
                    localf.z = frontierlocal.z;
                    aka.point.x = frontierlocal.x;
                    aka.point.y = frontierlocal.y;
                    aka.point.z = frontierlocal.z;
                    aka.local.data = true;       

                    psi = atan2((frontierlocal.y - m_currentPose.pose.pose.position.y),
                                        (frontierlocal.x - m_currentPose.pose.pose.position.x));

                    localfpose.pose.position.x = frontierlocal.x;
                    localfpose.pose.position.y = frontierlocal.y;
                    localfpose.pose.position.z = frontierlocal.z;

                    localfpose.pose.orientation.x = 0;
                    localfpose.pose.orientation.y = 0;
                    localfpose.pose.orientation.z = psi;
                    localfpose.pose.orientation.w = 0;

                    } else {

                    //If the point is out of field of view we mark it as a global frontier

                        global.push_back(frontier);

                        geometry_msgs::Point frontierglobal = FrontierSelection::weightGlobalFrontier(global);

                        globalf.x = frontierglobal.x;
                        globalf.y = frontierglobal.y;
                        globalf.z = frontierglobal.z;

                        aka_g.point.x = frontierglobal.x;
                        aka_g.point.y = frontierglobal.y;
                        aka_g.point.z = frontierglobal.z;
                        aka_g.local.data = false;

                        psi = atan2((frontierglobal.y - m_currentPose.pose.pose.position.y),
                                         (frontierglobal.x - m_currentPose.pose.pose.position.x));

                        globalfpose.pose.position.x = frontierglobal.x;
                        globalfpose.pose.position.y = frontierglobal.y;
                        globalfpose.pose.position.z = frontierglobal.z;

                        globalfpose.pose.orientation.x = 0;
                        globalfpose.pose.orientation.y = 0;
                        globalfpose.pose.orientation.z = psi;
                        globalfpose.pose.orientation.w = 0;

                    }
                }
            } 

        
        origin = octomap::point3d(m_currentPose.pose.pose.position.x,
                                         m_currentPose.pose.pose.position.y,
                                                 m_currentPose.pose.pose.position.z);
        lastPoint = octomap::point3d(minFrontier.x, minFrontier.y, minFrontier.z);

        // Here We Publish Next Best Frontier

        if (start_flag) {
            m_frontierPub.publish(localf);
            pub.x = localf.x;
            pub.y = localf.y;
            pub.z = localf.z;
            m_pub_frontier.publish(aka);
            pub.x = aka.point.x;
            pub.y = aka.point.y;
            pub.z = aka.point.z;
            // m_frontierPubPose.publish(localfpose);
            ROS_INFO("Local Frontier Published");
            start_flag = false;
        }

        pubpt = octomap::point3d(pub.x, pub.y, pub.z);
        reach_dis = abs(calculateDistance(origin, pubpt));
        if (reach_dis < 3) {
            next_pub = true;
        } else {
            next_pub = false;
        }
            
        if (next_pub) {
            switch (local.size() > 0)
            {
            case 1:
                
                m_frontierPub.publish(localf);
                pub.x = localf.x;
                pub.y = localf.y;
                pub.z = localf.z;
                m_pub_frontier.publish(aka);
                pub.x = aka.point.x;
                pub.y = aka.point.y;
                pub.z = aka.point.z;
                // m_frontierPubPose.publish(localfpose);
                ROS_INFO("Local Frontier Published");
                break;
                
            
            case 0:
                m_frontierPub.publish(globalf);
                m_frontierPubPose.publish(globalfpose);
                pub.x = globalfpose.pose.position.x;
                pub.y = globalfpose.pose.position.y;
                pub.z = globalfpose.pose.position.z;
                ROS_INFO("Global Frontier Published");
                break;
            }
        }

    }

    geometry_msgs::Point FrontierSelection::weightLocalFrontier(std::list<geometry_msgs::Point> local) {

        origin = octomap::point3d(m_currentPose.pose.pose.position.x,
                                         m_currentPose.pose.pose.position.y, 
                                                m_currentPose.pose.pose.position.z);
        minHeightDiff = 999999;
        minScore = 100000000;
        maxScore = -1000;
        midcost = 1500;
        myScore = 1500;

        for (geometry_msgs::Point localfrontier : local) {

            frontierPoint = octomap::point3d(localfrontier.x, localfrontier.y, localfrontier.z);

            dronepose = octomap::point3d(m_currentPose.pose.pose.position.x, m_currentPose.pose.pose.position.y, 0);

            distance = calculateDistance(frontierPoint, origin);
            heightDifference = abs((localfrontier.z - origin.z()));
            
            head = (atan2((frontierPoint.y() - dronepose.y()), (frontierPoint.x() - dronepose.x())) * (180/3.14159));

            angle = head - yaw_angle;

            score = angle;

            if ( (localfrontier.z > 2) || (localfrontier.z < 0.8)) {

                score = score + 10000;

            } else {

                score = score;

            }

            if (score < 0) {

                score = -score;
                
                if (score < minScore) {
                    
                    minScore = score;
                    minPoint = localfrontier;
                }
            }

            if (score > 0) {
                
                if (score < minScore) {
                    
                    minScore = score;
                    minPoint = localfrontier;
                }
            }            

        }
        
        return minPoint;

    }


    geometry_msgs::Point FrontierSelection::weightGlobalFrontier(std::list<geometry_msgs::Point> frontiers) {
        //Create points
        origin = octomap::point3d(m_currentPose.pose.pose.position.x, 
                                        m_currentPose.pose.pose.position.y, 
                                                m_currentPose.pose.pose.position.z);
        
    
        minScore = 99999999999;
        score;

        for (geometry_msgs::Point frontier : frontiers) {

            frontierPoint = octomap::point3d(frontier.x, frontier.y, frontier.z);

            heightDifference = abs(frontier.z - origin.z());

            distanceValue = abs((calculateDistance(origin, frontierPoint)));

            head = (atan2((frontierPoint.y() - dronepose.y()), 
                                    (frontierPoint.x() - dronepose.x())) * (180/3.14159));

            angle = head - yaw_angle;

            score = distanceValue;

            if ((frontier.z > 2) || (frontier.z < 1)) {

                score = abs(score) + 1000;
            
            } else {

                score = score;
            }

            if (score < 0) {

                score = -score;
                
                if (score < minScore) {
                    
                    minScore = score;
                    minPoint = frontier;
                }
            }

            if (score > 0) {
                
                if (score < minScore) {
                    
                    minScore = score;
                    minPoint = frontier;
                }
            }

        }

        return minPoint;
    }


    double FrontierSelection::calculateDistance(octomap::point3d origin, octomap::point3d point) {
        double distance = (point-origin).norm();
        return distance;
    }

    
    void FrontierSelection::updateCurrentPosition(const nav_msgs::Odometry::ConstPtr& pose) {
        // dis = 0;
        if (firstTime) {
            m_prevPose = *pose;
            firstTime = false;
        }

        else {

        //Update the current position.
        m_currentPose = *pose;


        newPoint = octomap::point3d(pose->pose.pose.position.x, 
                                            pose->pose.pose.position.y, pose->pose.pose.position.z);
        oldPoint = octomap::point3d(m_prevPose.pose.pose.position.x, 
                                            m_prevPose.pose.pose.position.y, m_prevPose.pose.pose.position.z);

        dis = sqrt((oldPoint.y() - newPoint.y()) * (oldPoint.y() - newPoint.y()) 
                                    + (oldPoint.x() - newPoint.x()) * (oldPoint.x() - newPoint.x()));

        if (dis > 0.6) {

            m_prevPose = *pose;
        
        }        
        if (dis > 0.5) {

            move_flag = true;
            
        } 
            else {
            move_flag = false;
        }

        }

        const Eigen::Quaterniond attitude   {

            pose->pose.pose.orientation.w, pose->pose.pose.orientation.x,
            pose->pose.pose.orientation.y, pose->pose.pose.orientation.z

            };

        const Eigen::Vector3d euler = quaternion_to_rpy(attitude);

        yaw_angle = euler.z() * (180/3.14159);

        heading.x() = cos(yaw_angle);
        heading.y() = sin(yaw_angle);
        heading.z() = 0;
        
    }
    


     void FrontierSelection::publishFrontier(visualization_msgs::MarkerArray frontierVis) {

        if (frontierVis.markers.size() > 0){
            for (unsigned i= 0; i < frontierVis.markers.size(); ++i){
                frontierVis.markers[i].header.frame_id = m_worldFrameId;
                frontierVis.markers[i].header.stamp = ros::Time::now();
                frontierVis.markers[i].ns = i;
                frontierVis.markers[i].id = i;
                frontierVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
                frontierVis.markers[i].scale.x = m_res;
                frontierVis.markers[i].scale.y = m_res;
                frontierVis.markers[i].scale.z = m_res;
                if (frontierVis.markers[i].points.size() > 0)
                    frontierVis.markers[i].action = visualization_msgs::Marker::ADD;
                else
                    frontierVis.markers[i].action = visualization_msgs::Marker::DELETE;
            }
        }
    }
     
