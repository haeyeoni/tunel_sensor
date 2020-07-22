#include "imu_process.h"

using namespace sensor_msgs;

using namespace geometry_msgs;
using namespace message_filters;

void callback(const geometry_msgs::TwistStamped::ConstPtr& wheel_odom, ros::Publisher publisher){
    geometry_msgs::TwistStamped odometry;
    odometry = *wheel_odom;
    odometry.header.frame_id ="odom";
    odometry.header.frame_id ="odom";
    odometry.child_frame_id = "base_link";
}


/*void callback(const Vector3Stamped::ConstPtr& imu_accel, const Vector3Stamped::ConstPtr& imu_gyro, ros::Publisher publisher)
{
    	sensor_msgs::Imu imu_topic;
	geometry_msgs::Vector3 gyro_vector = imu_gyro->vector;
	geometry_msgs::Vector3 accel_vector = imu_accel->vector;

	imu_topic.angular_velocity = gyro_vector;
	imu_topic.linear_acceleration = accel_vector;	
	
	double yaw = gyro_vector.z;
	double pitch = gyro_vector.y;
	double roll = gyro_vector.x;	

	double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);


	imu_topic.orientation.w = cr * cp * cy + sr * sp * sy;
    	imu_topic.orientation.x = sr * cp * cy - cr * sp * sy;
    	imu_topic.orientation.y = cr * sp * cy + sr * cp * sy;
    	imu_topic.orientation.z = cr * cp * sy - sr * sp * cy; 
	imu_topic.header.frame_id = "imu";
	publisher.publish(imu_topic);
}
*/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  ros::NodeHandle nh;
  ros::Publisher publisher = nh.advertise<sensor_msgs::Imu> ("imu_topic", 10);
  
  ros::Subscriber<geometry_msgs::TwistStamped> wheel_odom(nh, "/wheel_odom", 10);

//  message_filters::Subscriber<Vector3Stamped> accel_sub(nh, "/imu/accel", 1);
//  message_filters::Subscriber<Vector3Stamped> gyro_sub(nh, "/imu/gyro", 1);
//  TimeSynchronizer<Vector3Stamped, Vector3Stamped> sync(accel_sub, gyro_sub, 10);
//  sync.registerCallback(boost::bind(&callback, _1,_2,publisher));
  ros::spin();

  return 0;
}

