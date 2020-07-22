#include "line_ransac.h"
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PCLPointCloud2 PointCloud2;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud2::Ptr PointCloud2Ptr;


void callback(ros::Publisher publisher){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 1.0;
	cmd_vel.angular.z = 0.0;
    publisher.publish(cmd_vel);
}

void read_nearest


int main(int argc, char** argv)
{
  	ros::init(argc, argv, "talker");

  	ros::NodeHandle nh;
  	ros::Subscriber<geometry_msgs::TwistStamped> rp_subscribe = nh.subscribe("/scan", 10, extract_points);
  	
  	ros::Publisher cmd_publisher = nh.advertise<geometry_msgs::Twist> ("cmd_vel", 10);
	ros::Publisher nearest_publisher = nh.advertise<sensor_msgs::PointCloud2> ("line_points
    while(ros::ok()) {
        ros::spinOnce();
		callback(publisher);
    }

  return 0;
}
