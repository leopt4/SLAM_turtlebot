#!/usr/bin/python
import tf
import rospy
import numpy as np
from sensor_msgs.msg import JointState
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA 
from untils.Pose import *

wheelBase   = 0.235
wheelRadius = 0.035
odom_freq   = 0.1
odom_window = 100000.0
Qk = np.diag(np.array([0.01 ** 2, 0.001 ** 2, np.deg2rad(0.1) ** 2]))  # covariance of simulated displacement noise

def f(xk_1, uk):
    posBk_1 = Pose3D(xk_1[0:3])
    xk_bar  = posBk_1.oplus(uk)
    return xk_bar

def Jfx(xk_1, uk):
    posBk_1 = Pose3D(xk_1[0:3])
    J=posBk_1.J_1oplus(uk)
    return J

def Jfw(xk_1):
    posBk_1 = Pose3D(xk_1[0:3])
    J = posBk_1.J_2oplus()
    return J


def Prediction(uk, Qk, xk_1=None, Pk_1=None):
    """
    Prediction step of the EKF. It calls the motion model and its Jacobians to predict the state vector and its covariance matrix.

    :param uk: input vector
    :param Qk: covariance matrix of the noise vector
    :param xk_1: previous mean state vector. By default it is taken from the class attribute. Otherwise it updates the class attribute.
    :param Pk_1: covariance matrix of the previous state vector. By default it is taken from the class attribute. Otherwise it updates the class attribute.
    :return xk_bar, Pk_bar: predicted mean state vector and its covariance matrix. Also updated in the class attributes.
    """

    # KF equations begin here
    # TODO: To be implemented by the student
    # Predict states
    xk_bar = f(xk_1, uk)
    # Predict covariance
    Ak          = Jfx(xk_1, uk)
    Wk          = Jfw(xk_1)

    Pk_bar = Ak @ Pk_1 @ Ak.T + Wk @ Qk @ Wk.T

    return xk_bar, Pk_bar

class encoderReading:
    def __init__(self):
        self.position   = 0.0
        self.velocity   = 0.0
        self.stamp      = None

class Odometry:
    def __init__(self, odom_topic) -> None:

        # PUBLISHERS
        # Publisher for sending Odometry
        # self.odom_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        # Publisher for visualizing the path to with rviz
        # self.marker_pub = rospy.Publisher('~path_marker', Marker, queue_size=1)
        self.point_marker_pub = rospy.Publisher('~point_marker', Marker, queue_size=1)

        # self.tf_world_base_footprint_pub = rospy.Publisher('~point_marker', tf, queue_size=1)

        # SUBSCRIBERS
        self.odom_sub       = rospy.Subscriber(odom_topic, JointState, self.get_odom) 

        # TIMERS
        # Timer for displacement reset
        rospy.Timer(rospy.Duration(odom_window), self.reset_displacement)

        self.synchronized_velocity  = [0.0, 0.0]
        self.synchronized_stamp     = None
        self.inited_displacement    = False
        self.left_reading           = encoderReading()
        self.right_reading          = encoderReading()

        # Init computing displacement
        while True:
            if self.synchronized_stamp is not None:
                self.reset_displacement(0)
                break

        

    def get_odom(self, odom):
        # Check if encoder data is for the left wheel
        if 'turtlebot/wheel_left_joint' in odom.name:
            self.left_reading.position  = odom.position[0]
            self.left_reading.velocity  = odom.velocity[0]
            self.left_reading.stamp     = odom.header.stamp
        # Check if encoder data is for the right wheel
        elif 'turtlebot/wheel_right_joint' in odom.name:
            self.right_reading.position = odom.position[0]
            self.right_reading.velocity = odom.velocity[0]
            self.right_reading.stamp    = odom.header.stamp
        
        # Synchronize encoder data if readings for both wheels are available
        if self.left_reading.stamp is not None and self.right_reading.stamp is not None:
            next_synchronized_stamp     = 0.5 * ((self.left_reading.stamp.secs + self.right_reading.stamp.secs) + (self.left_reading.stamp.nsecs + self.right_reading.stamp.nsecs)/1e9)  
            # Compute displacement
            if self.synchronized_stamp is not None and self.inited_displacement is True:
                delta_t = next_synchronized_stamp - self.synchronized_stamp
                uk = self.compute_displacement(delta_t)
                self.xk, self.P = Prediction(uk, Qk, self.xk, self.P)
                self.displacement += uk
                self.publish_point(self.xk[0:2])

            self.synchronized_stamp     = next_synchronized_stamp
            # Synchronize encoder readings here
            # For demonstration, let's assume the readings are already synchronized
            self.synchronized_velocity  = [self.left_reading.velocity, self.right_reading.velocity]
            # Publish synchronized data or use it in your control algorithm

            # Reset readings for next iteration
            self.left_reading.stamp     = None
            self.right_reading.stamp    = None

            return True
        
        return False
    
    def compute_displacement(self, delta_t):
        d_L = self.synchronized_velocity[0] * wheelRadius * delta_t
        d_R = self.synchronized_velocity[1] * wheelRadius * delta_t
        # Compute displacement of the center point of robot between k-1 and k
        d       = (d_L + d_R) / 2.
        # Compute rotated angle of robot around the center point between k-1 and k
        delta_theta_k   = np.arctan2(d_R - d_L, wheelBase)

        # Compute xk from xk_1 and the travel distance and rotated angle. Got the equations from chapter 1.4.1: Odometry 
        uk              = np.array([[d],
                                    [0],
                                    [delta_theta_k]])
        
        return uk

    def reset_displacement(self, event):
        self.xk           = np.array([[0.0],
                                      [0.0],
                                      [0.0]])
        self.P            = np.zeros((3, 3))

        self.displacement = np.array([[0.0],
                                      [0.0],
                                      [0.0]])
        self.inited_displacement = True

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

    def spin(self):
        pass

if __name__ == '__main__':
    rospy.init_node('odom_publisher')
    node = Odometry('/turtlebot/joint_states')	
    
    rate = rospy.Rate(odom_freq)
    while not rospy.is_shutdown():
        node.spin()
        rate.sleep()