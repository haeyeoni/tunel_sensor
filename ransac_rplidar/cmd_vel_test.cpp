#include "cmd_vel_test.h"

class LineExtractRP
{
public:
    LineExtractRP() {
		this->subscriber = this->nh.subscribe("/scan_rp", 10, &LineExtractRP::LineExtract, this);
		this->pub_nearest = this->nh.advertise<sensor_msgs::PointCloud2> ("nearest_point", 10);
	 	this->pub_ref = this->nh.advertise<sensor_msgs::PointCloud2> ("reference_point", 10);
		this->pub_points = this->nh.advertise<sensor_msgs::PointCloud2> ("points_msg", 10);
		this->pub_line = this->nh.advertise<sensor_msgs::PointCloud2>("cluster_line", 10);
    };
	
    void LineExtract(const sensor_msgs::LaserScan::ConstPtr& scan_in) {
		// publish message
		float Width = 1.0;
		sensor_msgs::PointCloud2 nearest_point;
		sensor_msgs::PointCloud2 reference_point;      
		sensor_msgs::PointCloud2 points_msg;
		sensor_msgs::PointCloud2 points_line;
		
		sensor_msgs::PointCloud2 cloud_temp;      

		PointCloudPtr cloud(new PointCloud); //data cloud
		PointCloud2Ptr cloud2(new PointCloud2); // data cloud (2)
		PointCloudPtr cloud_inrange(new PointCloud); // cropped cloud

		PointCloud closest;
		PointCloud filtered_cloud;
		PointCloud point_set;
		PointCloud cluseter_line;
		
		// conversions from laser to pointcloud 2
		projector_.projectLaser(*scan_in, cloud_temp);
		pcl_conversions::toPCL(cloud_temp, *cloud2); // save cloud message to cloud2
		pcl::fromPCLPointCloud2(*cloud2, *cloud); 

		// crop pointcloud (cloud -> cloud_inrange) 
		pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condition(new pcl::ConditionAnd<pcl::PointXYZ> ());
		range_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -Width)));
		range_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, Width)));
		range_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 0.0)));

		pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
		condrem.setInputCloud(cloud);
		condrem.setCondition(range_condition);
		condrem.setKeepOrganized(true);
		condrem.filter(*cloud_inrange);


		// extract line (input cloud_inragne) 

		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_LINE);
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.5);
		seg.setInputCloud(cloud_inrange);
		seg.segment(*inliers, *coefficients);
		extract.setInputCloud(cloud_inrange);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_inrange);

		// line clustering

		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_cluster(new pcl::search::KdTree<pcl::PointXYZ>);
		tree_cluster->setInputCloud(cloud_inrange);
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setInputCloud(cloud_inrange);
		ec.setClusterTolerance(0.05);
		ec.setMinClusterSize(30);
		ec.setMaxClusterSize(1000);;
		ec.setSearchMethod(tree_cluster);
		ec.extract(cluster_indices);

		// extract first clustering
		int j = 0;
		std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			if (j == 0) {
				for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
				{		
					cloud_cluster->points.push_back (cloud_inrange->points[*pit]);
				}
				cloud_cluster->width = cloud_cluster->points.size ();
				cloud_cluster->height = 1;
				cloud_cluster->is_dense = true;
			}
			j++;

		}

		// find nearest point from the origin
		if((*cloud_cluster).size()>0){
			cout<<"start: "<<cloud_cluster->points[0].x<<", "<<cloud_cluster->points[0].y<<endl;
			cout<<"End: "<<cloud_cluster->points[(*cloud_cluster).size()-1].x<<", "<<cloud_cluster->points[(*cloud_cluster).size()-1].y<<endl;

			pcl::PointXYZ origin(0, 0, 0);	
			pcl::KdTree<pcl::PointXYZ>::Ptr tree_(new pcl::KdTreeFLANN<pcl::PointXYZ>);
			tree_->setInputCloud(cloud_cluster);
			std::vector<int> nn_indices(1);
			std::vector<float> nn_dists(1);
			tree_->nearestKSearch(origin, 1, nn_indices, nn_dists);
				closest.push_back(cloud_cluster->points[nn_indices[0]]);

			point_set.push_back(closest[0]);
			
			float current_x = cloud_cluster->points[nn_indices[0]].x;
			float current_y = cloud_cluster->points[nn_indices[0]].y;
			float threshold = 0.5;
			float threshold_y = 0.5;
			float sum_x = 0;
			float sum_y = 0;
			int num_points = 0;

			for (int i = 0; i < (*cloud_cluster).size(); i++) {
				filtered_cloud.push_back(cloud_cluster->points[i]);
				sum_x += cloud_cluster->points[i].x;
				sum_y += cloud_cluster->points[i].y;
				num_points ++;
			}

			PointCloud reference_cloud;
			pcl::PointXYZ reference (sum_x / (float)num_points, sum_y / (float)num_points, 0);	
				
			reference_cloud.push_back(reference);
			point_set.push_back(reference);
			
			pcl::toROSMsg(closest, nearest_point);
			pcl::toROSMsg((*cloud_cluster), points_line);
			pcl::toROSMsg(reference_cloud, reference_point);
			pcl::toROSMsg(point_set, points_msg);
				
			reference_point.header.frame_id = "laser";
			nearest_point.header.frame_id = "laser";
			points_line.header.frame_id = "laser";
			points_msg.header.frame_id = "laser";
				
			this->pub_nearest.publish(nearest_point);// current position		
			this->pub_ref.publish(reference_point);
			this->pub_points.publish(points_msg);
			this->pub_line.publish(points_line);
		}
    }
    ~LineExtractRP(){
    }
    private:
        ros::NodeHandle nh;
		ros::Subscriber subscriber;
        ros::Publisher pub_nearest;
        ros::Publisher pub_ref;
        ros::Publisher pub_points;
		ros::Publisher pub_line;
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
