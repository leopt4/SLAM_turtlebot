#!/usr/bin/env python

import rospy
import rosbag
from geometry_msgs.msg import PoseArray
from std_msgs.msg import String

from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image

import rospkg

def joint_states_callback(msg):
    global js_bag
    js_bag.write('/turtlebot/joint_states', msg)

def odom_ground_truth_callback(msg):
    global gt_bag
    gt_bag.write('/turtlebot/kobuki/odom_ground_truth', msg)

def keyframes_deadReckoning_callback(msg):
    global kf_dr_bag
    kf_dr_bag.write('/keyframes_deadReckoning', msg)

def pc_callback(msg):
    global pc_bag
    pc_bag.write('/pc', msg)

def keyframes_callback(msg):
    global kf_bag
    kf_bag.write('/keyframes', msg)

def odom_callback(msg):
    global odom_bag
    odom_bag.write('/odom', msg)

def camera_info_callback(msg):
    global ci_bag
    ci_bag.write('/turtlebot/kobuki/realsense/color/camera_info', msg)

def image_depth_callback(msg):
    global id_bag
    id_bag.write('/turtlebot/kobuki/realsense/depth/image_depth', msg)

def main():
    # Initialize the ROS node
    rospy.init_node('bag_node')

    # Initialize the ROS package manager
    rospack = rospkg.RosPack()
    # Get the path of the package
    package_path = str(rospack.get_path('turtlebot_graph_slam'))
    
    # Create a ROS bag
    global js_bag
    global odom_bag
    global pc_bag
    global gt_bag
    global kf_dr_bag
    global kf_bag
    global ci_bag
    global id_bag

    # Define the paths to save the bag files
    js_bag_dir         = package_path + '/logs/bag/joint_states.bag'
    odom_bag_dir       = package_path + '/logs/bag/odom.bag'
    pc_bag_dir         = package_path + '/logs/bag/point_cloud.bag'
    gt_bag_dir         = package_path + '/logs/bag/ground_truth.bag'
    kf_dr_bag_dir      = package_path + '/logs/bag/keyframe_deadReckon.bag'
    kf_bag_dir         = package_path + '/logs/bag/keyframe_scan.bag'
    ci_bag_dir         = package_path + '/logs/bag/camera_info.bag'
    id_bag_dir         = package_path + '/logs/bag/image_depth.bag'

    js_bag      = rosbag.Bag(js_bag_dir, 'w')
    odom_bag    = rosbag.Bag(odom_bag_dir, 'w')
    pc_bag      = rosbag.Bag(pc_bag_dir, 'w')
    gt_bag      = rosbag.Bag(gt_bag_dir, 'w')
    kf_dr_bag   = rosbag.Bag(kf_dr_bag_dir, 'w')
    kf_bag      = rosbag.Bag(kf_bag_dir, 'w')
    ci_bag      = rosbag.Bag(ci_bag_dir, 'w')
    id_bag      = rosbag.Bag(id_bag_dir, 'w')

    # Subscribe to the '/turtlebot/joint_states' topic
    rospy.Subscriber('/turtlebot/joint_states', JointState, joint_states_callback)
    
    # Subscribe to the '/odom' topic
    rospy.Subscriber('/odom', Odometry, odom_callback)

    # Subscribe to the '/pc' topic
    rospy.Subscriber('/pc', PointCloud2, pc_callback)

    # Subscribe to the '/turtlebot/kobuki/odom_ground_truth' topic
    rospy.Subscriber('/turtlebot/kobuki/odom_ground_truth', Odometry, odom_ground_truth_callback)
    
    # Subscribe to the '/keyframes_deadReckoning' topic
    rospy.Subscriber('/keyframes_deadReckoning', PoseArray, keyframes_deadReckoning_callback)

    # Subscribe to the '/keyframes' topic
    rospy.Subscriber('/keyframes', PoseArray, keyframes_callback)

    # Subscribe to the '/turtlebot/kobuki/realsense/color/camera_info' topic
    rospy.Subscriber('/turtlebot/kobuki/realsense/color/camera_info', CameraInfo, camera_info_callback)

    # Subscribe to the '/turtlebot/kobuki/realsense/depth/image_depth' topic
    rospy.Subscriber('/turtlebot/kobuki/realsense/depth/image_depth', Image, image_depth_callback)
    
    record_frequency = 1.0
    # Create a Rate object to control the recording frequency
    rate = rospy.Rate(record_frequency)
    
    # Record messages until the node is shutdown
    while not rospy.is_shutdown():
        # Record messages and sleep to achieve desired frequency
        # Note: This loop is typically left empty as messages are recorded in the callback function
        rate.sleep()
    

    # Close the ROS bag when the node is shutting down
    js_bag.close()
    odom_bag.close()
    pc_bag.close()
    gt_bag.close()
    kf_dr_bag.close()
    kf_bag.close()
    ci_bag.close()
    id_bag.close()



if __name__ == '__main__':
    main()
