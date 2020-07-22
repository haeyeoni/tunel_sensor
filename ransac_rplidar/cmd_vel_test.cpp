#include "line_ransac.h"


void callback(ros::Publisher publisher){
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 1.0;
	cmd_vel.angular.z = 0.0;
    publisher.publish(cmd_vel);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle nh;
  ros::Publisher publisher = nh.advertise<geometry_msgs::Twist> ("cmd_vel", 10);
  
  //ros::Subscriber<geometry_msgs::TwistStamped> wheel_odom(nh, "/wheel_odom", 10);

    while(ros::ok()) {
        ros::spinOnce();
	callback(publisher);
    }

  return 0;
}
