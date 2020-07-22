#!/usr/bin/env python
import rospy
import message_filters
import ros_numpy

from geometry_msgs.msg import Twist
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

def control_callback(ref, nearest):
#    print(ref[0])
    xyz_array = rosnumpy.point_cloud2.get_xyz_points(ref)
    
    print(xyz_array)
    
def callback(data):
    rospy.loginfo("data: %s", data)

def drive(linear, angular):
    # Initialize ROS message object
    twist = Twist()
    twist.linear.x = linear
    twist.angular.z = angular
    for _ in range(20): # repeat 20 times
        cmd_pub.publish(twist) # publish message
        rospy.sleep(0.1) # sleep for 100ms 

def listener():
    rospy.Subscriber("wheel_odom",TwistStamped,callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node("test_drive")
        cmd_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
#        drive(-0.1, 0.0)
#        drive(0.0, 0.0)
#        listener()
#drive(-0.2, 0.0)
#drive(0.0, 0.0)
#drive(0.0, 1.0)
#drive(0.0, 0.0)
#drive(0.0, -1.0)
#drive(0.0, 0.0)

        ref = message_filters.Subscriber('reference_point', PointCloud2)
        nearest = message_filters.Subscriber('nearest_point', PointCloud2)
        ts = message_filters.TimeSynchronizer([ref, nearest], 10)
        ts.registerCallback(control_callback)
        rospy.spin()

    except rospy.ROSInterruptException:
        pass


