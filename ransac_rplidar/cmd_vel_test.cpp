#include "cmd_vel_test.h"

class LineExtractRP
{
public:
    LineExtractRP() {
		this->subscriber = this->nh.subscribe("/scan", 10, &LineExtractRP::LineExtract, this);
		this->pub_nearest = this->nh.advertise<sensor_msgs::PointCloud2> ("nearest_point", 10);
	 	this->pub_ref = this->nh.advertise<sensor_msgs::PointCloud2> ("reference_point", 10);
    	this->pub_points = this->nh.advertise<sensor_msgs::PointCloud2> ("points_msg", 10);
    };

    void LineExtract(const sensor_msgs::LaserScan::ConstPtr& scan_in) {
        sensor_msgs::PointCloud2 nearest_point;
		sensor_msgs::PointCloud2 reference_point;
		sensor_msgs::PointCloud2 cloud_temp;      
		sensor_msgs::PointCloud2 points_msg;      

		PointCloudPtr cloud(new PointCloud); 
		PointCloud2Ptr cloud2(new PointCloud2);
		PointCloud closest;
		PointCloud filtered_cloud;
		PointCloud point_set;
		
		projector_.projectLaser(*scan_in, cloud_temp);
		pcl_conversions::toPCL(cloud_temp, *cloud2); // save cloud message to cloud2
		pcl::fromPCLPointCloud2(*cloud2, *cloud); 

		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		pcl::ExtractIndices<pcl::PointXYZ> extract;

		// extract vertical walls
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_LINE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.05);
		seg.setInputCloud(cloud);
		seg.segment(*inliers, *coefficients);
		extract.setInputCloud(cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud);

		// find nearest point from the origin
		
		pcl::PointXYZ origin(0, 0, 0);
				
		pcl::KdTree<pcl::PointXYZ>::Ptr tree_(new pcl::KdTreeFLANN<pcl::PointXYZ>);
	    tree_->setInputCloud(cloud);
	    std::vector<int> nn_indices(1);
	    std::vector<float> nn_dists(1);
	    tree_->nearestKSearch(origin, 1, nn_indices, nn_dists);
	    
        closest.push_back(cloud->points[nn_indices[0]]);
		point_set.push_back(closest[0]);
		
        float current_x = cloud->points[nn_indices[0]].x;
		float threshold = 0.5;
		float sum_x = 0;
		float sum_y = 0;
		int num_points = 0;	
	    for (int i = 0; i < (*cloud).size(); i++) {
            bool inside_x = cloud->points[i].x < current_x + threshold/2 && cloud->points[i].x > current_x - threshold/2;
            if(inside_x) {
                filtered_cloud.push_back(cloud->points[i]);
				sum_x += cloud->points[i].x;
				sum_y += cloud->points[i].y;
				num_points ++;
            }
        }
		
		PointCloud reference_cloud;
        pcl::PointXYZ reference (sum_x / (float)num_points, sum_y / (float)num_points, 0);	
		
        reference_cloud.push_back(reference);
		point_set.push_back(reference);
        
        pcl::toROSMsg(closest, nearest_point);
		pcl::toROSMsg(reference_cloud, reference_point);
        pcl::toROSMsg(point_set, points_msg);
		
        reference_point.header.frame_id = "laser";
		nearest_point.header.frame_id = "laser";
		
        this->pub_nearest.publish(nearest_point);// current position		
		this->pub_ref.publish(reference_point);
        this->pub_points.publish(points_msg);
    }
    ~LineExtractRP(){
    }
    private:
        ros::NodeHandle nh;
        ros::Subscriber subscriber;
        ros::Publisher pub_nearest;
        ros::Publisher pub_ref;
        ros::Publisher pub_points;
        laser_geometry::LaserProjection projector_;
};

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

int main(int argc, char** argv)
{
  	ros::init(argc, argv, "talker");

    LineExtractRP LineExtractRP;
    Command Command;
    while(ros::ok()) {
        ros::spinOnce();
	}
  return 0;
}
