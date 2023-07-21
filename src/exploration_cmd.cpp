#include "exploration_cmd.h"

using namespace std;

ExplorationCMD::ExplorationCMD(ros::NodeHandle private_nh_)
    : m_nh(),
    cloud("/hummingbird/velodyne_points"),
    publishPointTopic("/hummingbird/reference"),
    odom("/hummingbird/ground_truth/odometry"),
    resolution(0.15),
    distance_threshold(1),
    num_opt_points(7),
    distanceBeforeSelectingNew(0.5),
    dt(0.5),
    field(10),
    stopTimeGlobalFrontier(2.0)
    {

         ros::NodeHandle private_nh = private_nh_;

         private_nh.param("point_cloud_topic", cloud, cloud);
         private_nh.param("distanceBeforeSelectingNew", distanceBeforeSelectingNew, distanceBeforeSelectingNew);
         private_nh.param("odom_topic", odom, odom);
         private_nh.param("field", field, field);
         private_nh.param("publish_point_topic", publishPointTopic, publishPointTopic);
         private_nh.param("max_velocity", max_velocity, 2.0);
         private_nh.param("max_acceleration", max_acceleration, 5.0);
         private_nh.param("resolution", resolution, resolution);
         private_nh.param("distance_threshold", distance_threshold, distance_threshold);
         private_nh.param("dt", dt, dt);
         private_nh.param("num_opt_points", num_opt_points, num_opt_points);
         private_nh.param("stopTimeGlobalFrontier", stopTimeGlobalFrontier, stopTimeGlobalFrontier);

         listener = new tf::TransformListener;

         //Publishers
         m_frontierVisPub = m_nh.advertise<visualization_msgs::MarkerArray>("selected_cells_vis_array", 5, true);
         nextPointPub = m_nh.advertise<geometry_msgs::PoseStamped>("/cmd/reference", 1, true);
         // nextDirectPointPub = m_nh.advertise<geometry_msgs::Pose>("/shafter3d/reference", 10, true);
         traj_marker_pub = m_nh.advertise<visualization_msgs::MarkerArray>("global_trajectory", 1, true);
         current_traj_pub = m_nh.advertise<visualization_msgs::MarkerArray>("optimal_trajectory", 1, true);
         occ_marker_pub = m_nh.advertise<visualization_msgs::Marker>("ring_buffer/occupied", 1, true);
         free_marker_pub = m_nh.advertise<visualization_msgs::Marker>("ring_buffer/free", 1, true);
         dist_marker_pub = m_nh.advertise<visualization_msgs::Marker>("ring_buffer/distance", 5);
         path_pub = m_nh.advertise<nav_msgs::Path>("exploration/path", 1);
         //Subscribers
         commandSub = new message_filters::Subscriber<std_msgs::String> (m_nh, "command", 1);
         pointSub = new message_filters::Subscriber<exploration::Frontier> (m_nh, "/cmd_frontier", 5);
         
         pointCloudSubscriber = new message_filters::Subscriber<sensor_msgs::PointCloud2> (m_nh, cloud, 5);
         m_tfPointCloudSubscriber = new tf::MessageFilter<sensor_msgs::PointCloud2> (*pointCloudSubscriber, m_tfListener, "world", 5);
         m_poseSubscriber = new message_filters::Subscriber<nav_msgs::Odometry> (m_nh, odom, 1);

         //Register subscribers to callback functions
         commandSub->registerCallback(boost::bind(&ExplorationCMD::commandHandler, this, _1));
         pointSub->registerCallback(boost::bind(&ExplorationCMD::pointHandler, this, _1));
         m_poseSubscriber->registerCallback(boost::bind(&ExplorationCMD::updateCurrentPosition, this, _1));
         m_tfPointCloudSubscriber->registerCallback(boost::bind(&ExplorationCMD::pointCloudCallback, this, _1));
         timer = m_nh.createTimer(ros::Duration(dt), boost::bind(&ExplorationCMD::sendCommandCallback, this, _1));
         timer.stop();

         //Initialize
         edrb.reset(new ewok::EuclideanDistanceRingBuffer<6>(resolution, 1.0));
         limits = Eigen::Vector4d (max_velocity, max_acceleration, 0, 0);
         frontierVis.markers.resize(1);





     }

/*********************************
 * Start and stop of exploration *
 *********************************/
void ExplorationCMD::commandHandler(const std_msgs::String::ConstPtr& message) {
   if(message->data == "explore") {
      startTime = ros::Time::now();
      explore = true;
   }
   if(message->data == "stop") {
      endTime = ros::Time::now();
      ROS_INFO("%f", endTime.toSec() - startTime.toSec());
      explore = false;
   }
}

/***********************************
 * Handles every new frontier point*
 ***********************************/
void ExplorationCMD::pointHandler(const exploration::Frontier::ConstPtr& point) {
   octomap::point3d currentPosition (m_currentPose.pose.pose.position.x, m_currentPose.pose.pose.position.y, m_currentPose.pose.pose.position.z);

   // if (explore) {
      frontierPoint = octomap::point3d(point->point.x, point->point.y, point->point.z);
      float distance = (frontierPoint - currentPosition).norm();
      octomath::Vector3 direction = (frontierPoint - currentPosition).normalize();
      // if(distance < distanceBeforeSelectingNew || firstTime) {
         // firstTime = false;
      

      //Start point
      float start_x;
      float start_y;
      float start_z;

      // if (point->local.data == true) {
         start_x = currentPosition.x()+(direction.x()*1.1);
         start_y = currentPosition.y()+(direction.y()*1.1);
         start_z = currentPosition.z();
         // cout << "X dirs dist" << (direction.x()*1.5) << std::endl;
         // cout << "Y dirs dist    " << (direction.y()*1.5) << std::endl;
         // start_x = currentPosition.x();
         // start_y = currentPosition.y();
         // start_z = currentPosition.z();


      // }
      // else {
      //    // ExplorationCMD::stopFunction();
      //    // ros::Duration(stopTimeGlobalFrontier).sleep();
      //    nav_msgs::Odometry::ConstPtr odometry = ros::topic::waitForMessage<nav_msgs::Odometry>(odom);
      //    start_x = odometry->pose.pose.position.x;
      //    start_y = odometry->pose.pose.position.y;
      //    start_z = odometry->pose.pose.position.z;
      // }

      float stop_x = point->point.x;
      float stop_y = point->point.y;
      float stop_z = point->point.z;

      ewok::Polynomial3DOptimization<10> to(limits*0.6);
      {
         typename ewok::Polynomial3DOptimization<10>::Vector3Array path;
         path.push_back(Eigen::Vector3d(start_x, start_y, start_z));
         path.push_back(Eigen::Vector3d(stop_x, stop_y, stop_z));
         traj = to.computeTrajectory(path);
         visualization_msgs::MarkerArray traj_marker;
         traj->getVisualizationMarkerArray(traj_marker, "gt", Eigen::Vector3d(1,0,1));
         traj_marker_pub.publish(traj_marker);

      }
      spline_optimization.reset(new ewok::UniformBSpline3DOptimization<6>(traj, dt));
      for (int i = 0; i < num_opt_points; i++) {
         spline_optimization->addControlPoint(Eigen::Vector3d(start_x, start_y, start_z));
      }
      spline_optimization->setNumControlPointsOptimized(num_opt_points);
      spline_optimization->setDistanceBuffer(edrb);
      spline_optimization->setDistanceThreshold(distance_threshold);
      spline_optimization->setLimits(limits);
      
      nav_msgs::Odometry::ConstPtr odometry = ros::topic::waitForMessage<nav_msgs::Odometry>(odom);

      my_current_pos = octomap::point3d(odometry->pose.pose.position.x, 
                                          odometry->pose.pose.position.y, 
                                             odometry->pose.pose.position.z);

      
      

      std_msgs::ColorRGBA color;
      color.a = 0.5;
      color.r = 0; color.g = 0; color.b = 1;

      geometry_msgs::Point selectedFrontier;
      selectedFrontier.x = point->point.x;
      selectedFrontier.y = point->point.y;
      selectedFrontier.z = point->point.z;

      mine = octomap::point3d(selectedFrontier.x, selectedFrontier.y, selectedFrontier.z);

      double rang = (mine - my_current_pos).norm();

      if (rang < field) {

         timer.start();
      }

      frontierVis.markers[0].points.push_back(selectedFrontier);
      frontierVis.markers[0].colors.push_back(color);
      if (frontierVis.markers.size() > 0){
         for (unsigned i= 0; i < frontierVis.markers.size(); ++i){
               frontierVis.markers[i].header.frame_id = "world";
               frontierVis.markers[i].header.stamp = ros::Time::now();
               frontierVis.markers[i].ns = i;
               frontierVis.markers[i].id = i;
               frontierVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
               frontierVis.markers[i].scale.x = 0.5;
               frontierVis.markers[i].scale.y = 0.5;
               frontierVis.markers[i].scale.z = 0.5;
               if (frontierVis.markers[i].points.size() > 0)
                  frontierVis.markers[i].action = visualization_msgs::Marker::ADD;
               else
                  frontierVis.markers[i].action = visualization_msgs::Marker::DELETE;
         }
         m_frontierVisPub.publish(frontierVis);
      }

      // }
   // }
}

void ExplorationCMD::updateCurrentPosition(const nav_msgs::Odometry::ConstPtr& pose) {
   //Update the current position.
   m_currentPose = *pose;

   geometry_msgs::PoseStamped stamped_pose;
   stamped_pose.pose.position.x = pose->pose.pose.position.x;
   stamped_pose.pose.position.y = pose->pose.pose.position.y;
   stamped_pose.pose.position.z = pose->pose.pose.position.z;
   stamped_pose.header.stamp = ros::Time::now();
   stamped_pose.header.frame_id = "world";

   path.poses.push_back(stamped_pose);
   path.header.stamp = ros::Time::now();
   path.header.frame_id = "world";

   path_pub.publish(path);
}

void ExplorationCMD::pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &msg) {
   tf::StampedTransform sensorToWorldTf;

   try {
      m_tfListener.lookupTransform("world", msg->header.frame_id, msg->header.stamp, sensorToWorldTf);
   } catch(tf::TransformException& ex) {
      ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
      return;
   }

   Eigen::Affine3d dT_w_c;
   tf::transformTFToEigen(sensorToWorldTf, dT_w_c);

   Eigen::Affine3f T_w_c = dT_w_c.cast<float>();

   ewok::EuclideanDistanceRingBuffer<6>::PointCloud cloud1;
   pcl::PointCloud<pcl::PointXYZ> cloud;
   pcl::fromROSMsg(*msg, cloud);

   for (pcl::PointCloud<pcl::PointXYZ>::iterator it = cloud.begin(), end = cloud.end(); it != end; ++it)
    {

       if (it->x < field && it->x > -field)
         {
            if (it->y < field && it->y > -field)
          {
            if (it->z < field && it->z > -field)
          {
        Eigen::Vector4f p;
        //ROS_INFO("%f, %f, %f", cloud.points[nIndex].x, cloud.points[nIndex].y, cloud.points[nIndex].z );
        p[0] = it->x;
        p[1] = it->y;
        p[2] = it->z;
        p[3] = 1;

        p = T_w_c * p;

        cloud1.push_back(p);
        }
     }
       }
    }

  Eigen::Vector3f origin = (T_w_c * Eigen::Vector4f(0,0,0,1)).head<3>();

   if(!initialized) {
      Eigen::Vector3i idx;
      edrb->getIdx(origin, idx);

      ROS_INFO_STREAM("Origin: " << origin.transpose() << " idx " << idx.transpose());

      edrb->setOffset(idx);

      initialized = true;

    } else {
        Eigen::Vector3i origin_idx, offset, diff;
        edrb->getIdx(origin, origin_idx);
        offset = edrb->getVolumeCenter();
        diff = origin_idx - offset;

        while(diff.array().any()) {
            edrb->moveVolume(diff);
            offset = edrb->getVolumeCenter();
            diff = origin_idx - offset;
        }


    }
   edrb->insertPointCloud(cloud1, origin); //This kills me
   visualization_msgs::Marker m_occ, m_free;
   edrb->getMarkerOccupied(m_occ);
   edrb->getMarkerFree(m_free);

   m_occ.header.frame_id = "world";
   occ_marker_pub.publish(m_occ);
   
   m_free.header.frame_id = "world";
   free_marker_pub.publish(m_free);
   
}

void ExplorationCMD::sendCommandCallback(const ros::TimerEvent& event) {
   edrb->updateDistance();

   visualization_msgs::MarkerArray traj_marker;

   spline_optimization->optimize();

   nav_msgs::Odometry::ConstPtr odometry = ros::topic::waitForMessage<nav_msgs::Odometry>(odom);

   my_current_pos = octomap::point3d(odometry->pose.pose.position.x, 
                                       odometry->pose.pose.position.y, 
                                          odometry->pose.pose.position.z);

   Eigen::Vector3d pc = spline_optimization->getFirstOptimizationPoint();
   

   octomap::point3d first_opt = octomap::point3d(pc[0], pc[1], pc[2]);

   double d = abs((first_opt - my_current_pos).norm());


   double yaw_ref = atan2((pc[1] - odometry->pose.pose.position.y), (pc[0] - odometry->pose.pose.position.x));

    geometry_msgs::PoseStamped pp;
    pp.pose.position.x = pc[0];
    pp.pose.position.y = pc[1];
    pp.pose.position.z = pc[2];

    pp.pose.orientation.x = 0;
    pp.pose.orientation.y = 0;
    pp.pose.orientation.z = yaw_ref;
    pp.pose.orientation.w = 0;

   nextPointPub.publish(pp);

   // if (init_explore || d < 1.5) {
   //    nextPointPub.publish(pp);
   //    init_explore = false;
   // } 


      std_msgs::ColorRGBA color;
      color.a = 0;
      color.r = 1; color.g = 0; color.b = 0;

      spline_optimization->getMarkers(traj_marker);

      // traj_marker.markers[0].points.push_back(selectedFrontier);
      traj_marker.markers[0].colors.push_back(color);
      if (traj_marker.markers.size() > 0){
         for (unsigned i= 0; i < traj_marker.markers.size(); ++i){
               traj_marker.markers[i].header.frame_id = "world";
               traj_marker.markers[i].header.stamp = ros::Time::now();
               traj_marker.markers[i].ns = i;
               traj_marker.markers[i].id = i;
               traj_marker.markers[i].type = visualization_msgs::Marker::SPHERE_LIST;
               traj_marker.markers[i].scale.x = 0.15;
               traj_marker.markers[i].scale.y = 0.15;
               traj_marker.markers[i].scale.z = 0.15;
               if (traj_marker.markers[i].points.size() > 0)
                  traj_marker.markers[i].action = visualization_msgs::Marker::ADD;
               else
                  traj_marker.markers[i].action = visualization_msgs::Marker::DELETE;
         }
         current_traj_pub.publish(traj_marker);
      }

   //  traj_marker.header.frame_id = "world";
   //  current_traj_pub.publish(traj_marker);

    spline_optimization->addLastControlPoint();

    visualization_msgs::Marker m_dist;
    edrb->getMarkerDistance(m_dist, 0.5);

    m_dist.header.frame_id = "world";
    dist_marker_pub.publish(m_dist);
}
