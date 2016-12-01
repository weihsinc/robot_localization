#!/usr/bin/env python
import rospy
import roslib
import tf
import math
import numpy as np

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped

ftr_rpy_msg = Vector3Stamped();
nov_rpy_msg = Vector3Stamped();
xs_rpy_msg = Vector3Stamped();

filter_rpy_pub = rospy.Publisher('filter/rpy', Vector3Stamped, queue_size=1)
novatel_rpy_pub = rospy.Publisher('novatel/rpy', Vector3Stamped, queue_size=1)
xsens_rpy_pub = rospy.Publisher('xsens/rpy_transformed', Vector3Stamped, queue_size=1)

def filter_cb(msg):
    # Filter sensor frame: X-right, Y-forward, Z-up
    quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    x_rot = math.degrees(euler[0]);
    y_rot = math.degrees(euler[1]);
    z_rot = math.degrees(euler[2]);
    ftr_rpy_msg.vector.x = x_rot;
    ftr_rpy_msg.vector.y = y_rot;
    ftr_rpy_msg.vector.z = z_rot;
    ftr_rpy_msg.header = msg.header;
    filter_rpy_pub.publish(ftr_rpy_msg)

def novatel_cb(msg):
    # Novatel sensor frame: X-right, Y-forward, Z-up
    quaternion = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    x_rot = math.degrees(euler[0]);
    y_rot = math.degrees(euler[1]);
    z_rot = math.degrees(euler[2]);
    nov_rpy_msg.vector.x = x_rot;
    nov_rpy_msg.vector.y = y_rot;
    nov_rpy_msg.vector.z = z_rot;
    nov_rpy_msg.header = msg.header;
    novatel_rpy_pub.publish(nov_rpy_msg)

def xsens_cb(msg):
    # Xsens sensor frame: X-forward, Y-left, Z-up
    quaternion = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    # Transform from Xsens sensor frame (X-forward, Y-left, Z-up) to vehicle frame (X-right, Y-forward, Z-up)
    x_rot = math.degrees(-euler[1]); # new_x_rotation = -old_y_rotation
    y_rot = math.degrees(euler[0]);  # new_y_rotation = old_x_rotation
    z_rot = math.degrees(adjust_angle(euler[2] - np.pi/2)); # new_z_rotation = old_z_rotation - pi/2
    xs_rpy_msg.vector.x = x_rot;
    xs_rpy_msg.vector.y = y_rot;
    xs_rpy_msg.vector.z = z_rot;
    xs_rpy_msg.header = msg.header;
    xsens_rpy_pub.publish(xs_rpy_msg)

# Adjust the angle to be within the range from -pi to pi
def adjust_angle(angle):
    if angle < -np.pi:
        angle += 2*np.pi;
    elif angle > np.pi:
        angle -= 2*np.pi;
    return angle

def trace():
    rospy.init_node('rpy_publisher_node', anonymous=True)
    rospy.Subscriber("/odometry/filtered", Odometry, filter_cb) 
    rospy.Subscriber("/span/pose", Odometry, novatel_cb)
    rospy.Subscriber("/imu/data", Imu, xsens_cb)
    rospy.spin()

if __name__ == '__main__':
    try:
        trace()
    except rospy.ROSInterruptException:
        pass

