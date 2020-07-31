
#include "command_robot.h"
#include "cmd_vel_test.h"
using namespace COMMAND;

class Command
{
public:
    Command() {
		this->subscriber = this->nh.subscribe("/points_msg", 10, &Command::PublishCmd, this);
		this->publisher = this->nh.advertise<geometry_msgs::Twist> ("cmd_vel", 1);
	};

    void PublishCmd(const sensor_msgs::PointCloud2 &cloud_msg) {
        PointCloud2Ptr tmp_cloud(new PointCloud2);
        pcl_conversions::toPCL(cloud_msg, *tmp_cloud);
        PointCloudPtr data_cloud(new PointCloud); 
        pcl::fromPCLPointCloud2(*tmp_cloud, *data_cloud); 
        
        pcl::PointXYZ nearest_point = data_cloud->points[0];
        pcl::PointXYZ reference_point = data_cloud->points[1];

        geometry_msgs::Twist cmd_vel;
        float x = nearest_point.x - reference_point.x;
        float y = nearest_point.y - reference_point.y;
        cmd_vel.linear.x = x;
        cmd_vel.angular.z = y;
        this->publisher.publish(cmd_vel);	
    }
    ~Command(){
    }
    private:
        ros::NodeHandle nh;
        ros::Subscriber subscriber;
        ros::Publisher publisher;
};