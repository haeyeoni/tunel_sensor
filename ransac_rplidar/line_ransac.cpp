#include "line_ransac.h"

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PCLPointCloud2 PointCloud2;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud2::Ptr PointCloud2Ptr;

class LineExtractRP
{
public:
    LineExtractRP() {
		this->subscriber = this->nh.subscribe("/scan", 10, &LineExtractRP::LineExtract, this);
		this->publisher = this->nh.advertise<sensor_msgs::PointCloud2> ("line_points", 10);
		this->publisher2 = this->nh.advertise<sensor_msgs::PointCloud2> ("nearest_point", 10);
	 	//this->publisher3 = this->nh.advertise<std_msgs::Float64MultiArray> ("distances", 10);
	 	this->publisher4 = this->nh.advertise<sensor_msgs::PointCloud2> ("filtered_msg", 10);
	 	this->publisher5 = this->nh.advertise<sensor_msgs::PointCloud2> ("reference_point", 10);
    };

    void LineExtract(const sensor_msgs::LaserScan::ConstPtr& scan_in) {
        sensor_msgs::PointCloud2 line_points;
		sensor_msgs::PointCloud2 nearest_point;
		sensor_msgs::PointCloud2 filtered_msg;
		sensor_msgs::PointCloud2 reference_point;
		//std_msgs::Float64MultiArray distances;
		sensor_msgs::PointCloud2 cloud_temp;;      

		PointCloudPtr cloud(new PointCloud); 
		PointCloud2Ptr cloud2(new PointCloud2);
		PointCloud closest;
		PointCloud filtered_cloud;

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

		// publish the points in line
		pcl::toROSMsg(*cloud, line_points);
		line_points.header.frame_id = "laser";
		this -> publisher.publish(line_points);

		// find nearest point from the origin
		
		pcl::PointXYZ origin(0, 0, 0);
				
		pcl::KdTree<pcl::PointXYZ>::Ptr tree_(new pcl::KdTreeFLANN<pcl::PointXYZ>);
	    tree_->setInputCloud(cloud);
	    std::vector<int> nn_indices(1);
	    std::vector<float> nn_dists(1);
	    tree_->nearestKSearch(origin, 1, nn_indices, nn_dists);
	    //distances.data.clear();
		//pcl::PointXYZ current = cloud->points[nn_indices[0]];
        closest.push_back(cloud->points[nn_indices[0]]);
		float current_x = cloud->points[nn_indices[0]].x;
		float threshold = 0.5;
        //distances.data.push_back(nn_dists[0]);
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
		
		pcl::toROSMsg(closest, nearest_point);
		pcl::toROSMsg(filtered_cloud, filtered_msg);
		nearest_point.header.frame_id = "laser";
		filtered_msg.header.frame_id = "laser";
		PointCloud reference_cloud;
		pcl::PointXYZ reference (sum_x / (float)num_points, sum_y / (float)num_points, 0);	
		reference_cloud.push_back(reference);
		pcl::toROSMsg(reference_cloud, reference_point);
		reference_point.header.frame_id = "laser";
		this->publisher2.publish(nearest_point);// current position		
		this->publisher4.publish(filtered_msg);
		this->publisher5.publish(reference_point);
	
    }
    ~LineExtractRP(){
    }
    private:
        ros::NodeHandle nh;
        ros::Subscriber subscriber;
        ros::Publisher publisher;
        ros::Publisher publisher2;
        //ros::Publisher publisher3;
        ros::Publisher publisher4;
        ros::Publisher publisher5;

        laser_geometry::LaserProjection projector_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "talker");
    LineExtractRP LineExtractRP;
    while(ros::ok()) {
        ros::spinOnce();
    }
}
