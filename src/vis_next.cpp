#include <ros/ros.h>
#include <exploration/Frontier.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>

ros::Subscriber sub;
ros::Publisher vis_pub;

void pub(const geometry_msgs::PoseStampedConstPtr& msg)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world_shafter";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = msg->pose.position.x;
    marker.pose.position.y = msg->pose.position.y;
    marker.pose.position.z = msg->pose.position.z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.8;
    marker.scale.y = 0.8;
    marker.scale.z = 0.8;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    //only if using a MESH_RESOURCE marker type:
    marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";  
    vis_pub.publish( marker );

}

int main(int argc, char **argv) {
    ros::init(argc, argv, "vis_next");
    ros::NodeHandle nh_("~");
    
    vis_pub = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 1 );

    sub = nh_.subscribe<geometry_msgs::PoseStamped>("/frontier/pose", 1, pub);
    
        ros::spin();

    return 0;
}
