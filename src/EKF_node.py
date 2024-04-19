#!/usr/bin/python
import tf
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from untils.EKF_3DOF_InputDisplacement_Heading import *

from untils.odometry import *
from untils.magnetometer import *

from turtlebot_graph_slam.srv import ResetFilter, ResetFilterResponse

odom_freq   = 0.1
odom_window = 100000.0


class EKF:
    def __init__(self, odom_topic) -> None:
        self.current_pose           = None
        self.xk           = np.zeros((3, 1))
        self.Pk           = np.zeros((3, 3))
        self.yawOffset    = 0.0
        self.ekf_filter   = None

        # PUBLISHERS
        # Publisher for visualizing the path to with rviz
        self.point_marker_pub   = rospy.Publisher('~point_marker', Marker, queue_size=1)
        # Publisher for sending Odometry
        self.odom_pub           = rospy.Publisher('/odom', Odometry, queue_size=1)
        
        # SUBSCRIBERS
        self.odom_sub               = rospy.Subscriber(odom_topic, JointState, self.get_odom) 
        self.ground_truth_sub       = rospy.Subscriber('/turtlebot/kobuki/odom_ground_truth', Odometry, self.get_ground_truth) 
        
        # Init using sensors
        self.odom   = OdomData()
        self.mag    = Magnetometer()
        
        # Move
        while True:
            if self.current_pose is not None:
                # self.xk           = self.current_pose.reshape(3,1)
                # self.xk           = np.zeros((3, 1))
                # self.Pk           = np.zeros((3, 3))
                self.yawOffset    = self.current_pose[2]
                break
        
        # SERVICES
        reset_srv = rospy.Service('ResetFilter', ResetFilter, self.reset_filter)

        # TIMERS
        # Timer for displacement reset
        # rospy.Timer(rospy.Duration(odom_window), self.reset_filter)

        # Init EKF Filter
        self.ekf_filter = EKF_3DOF_InputDisplacement_Heading(self.xk, self.Pk, self.odom, self.mag)
    
    # Ground Truth callback: Gets current robot pose and stores it into self.current_pose. Besides, get heading as a measurement to update filter
    def get_ground_truth(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                            odom.pose.pose.orientation.y,
                                                            odom.pose.pose.orientation.z,
                                                            odom.pose.pose.orientation.w])
        self.current_pose = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, yaw])

        # Get heading as a measurement to update filter
        if self.mag.read_magnetometer(yaw-self.yawOffset) and self.ekf_filter is not None:
            self.ekf_filter.gotNewHeadingData()

    # Odometry callback: Gets encoder reading to compute displacement of the robot as input of the EKF Filter.
    # Run EKF Filter with frequency of odometry reading
    def get_odom(self, odom):
        # Read encoder
        if self.odom.read_encoder(odom) and self.ekf_filter is not None:
            self.ekf_filter.gotNewEncoderData()

        if self.ekf_filter is not None:
            # Run EKF Filter
            self.xk, self.Pk = self.ekf_filter.Localize(self.xk, self.Pk)

            # Publish rviz
            self.publish_point(self.xk[0:2])
            self.odom_path_pub()

    # Reset state and covariance of th EKF filter
    def reset_filter(self, request):
        # self.xk           = self.current_pose.reshape(3,1)
        self.yawOffset    += self.xk[2]
        self.xk           = np.zeros((3, 1))
        self.Pk           = np.zeros((3, 3))
        return ResetFilterResponse(request.reset_filter_requested)


    # Publish markers
    def publish_point(self,p):
        if p is not None:
            m = Marker()
            m.header.frame_id = 'world_ned'
            m.header.stamp = rospy.Time.now()
            m.ns = 'point'
            m.id = 0
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = p[0]
            m.pose.position.y = p[1]
            m.pose.position.z = 0.0
            m.pose.orientation.x = 0
            m.pose.orientation.y = 0
            m.pose.orientation.z = 0
            m.pose.orientation.w = 1
            m.scale.x = 0.05
            m.scale.y = 0.05
            m.scale.z = 0.05
            m.color.a = 1.0
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            self.point_marker_pub.publish(m)

    # Publish Filter results
    def odom_path_pub(self):
        # Transform theta from euler to quaternion
        quaternion = tf.transformations.quaternion_from_euler(0, 0, float((self.xk[2, 0])))  # Convert euler angles to quaternion

        # Publish predicted odom
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = "world_ned"
        odom.child_frame_id = "turtlebot/kobuki/predicted_base_footprint"


        odom.pose.pose.position.x = self.xk[0]
        odom.pose.pose.position.y = self.xk[1]

        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]

        odom.pose.covariance = list(np.array([[self.Pk[0, 0], self.Pk[0, 1], 0, 0, 0, self.Pk[0, 2]],
                                [self.Pk[1, 0], self.Pk[1,1], 0, 0, 0, self.Pk[1, 2]],
                                [0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0],
                                [0, 0, 0, 0, 0, 0],
                                [self.Pk[2, 0], self.Pk[2, 1], 0, 0, 0, self.Pk[2, 2]]]).flatten())

        # odom.twist.twist.linear.x = self.v
        # odom.twist.twist.angular.z = self.w

        self.odom_pub.publish(odom)

        tf.TransformBroadcaster().sendTransform((float(self.xk[0, 0]), float(self.xk[1, 0]), 0.0), quaternion, rospy.Time.now(), odom.child_frame_id, odom.header.frame_id)
        
    def spin(self):
        pass

if __name__ == '__main__':
    rospy.init_node('odom_publisher')
    node = EKF('/turtlebot/joint_states')	
    
    rate = rospy.Rate(odom_freq)
    while not rospy.is_shutdown():
        node.spin()
        rate.sleep()