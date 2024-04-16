#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
// #include "gtsam/navigation/PreintegrationBase.h"
// #include <gtsam/geometry/Pose3.h>
// #include <gtsam/geometry/Point3.h>
// #include <gtsam/geometry/Rot3.h>
// #include <gtsam/geometry/Pose3.h>

class ImuData {
public:
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
        // Accessing header
        header = msg->header;
        // Accessing orientation
        orientation = msg->orientation;
        // Accessing orientation_covariance array
        orientation_covariance = msg->orientation_covariance;
        // Accessing angular_velocity
        angular_velocity = msg->angular_velocity;
        // Accessing angular_velocity_covariance array
        angular_velocity_covariance = msg->angular_velocity_covariance;
        // Accessing linear_acceleration
        linear_acceleration = msg->linear_acceleration;
        // Accessing linear_acceleration_covariance array
        linear_acceleration_covariance = msg->linear_acceleration_covariance;
    }


private:
    // Class variables to store message data
    std_msgs::Header header;
    geometry_msgs::Quaternion orientation;
    boost::array<double, 9> orientation_covariance;
    geometry_msgs::Vector3 angular_velocity;
    boost::array<double, 9> angular_velocity_covariance;
    geometry_msgs::Vector3 linear_acceleration;
    boost::array<double, 9> linear_acceleration_covariance;
};

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "IMU_node");

    // Create a node handle
    ros::NodeHandle nh;

    // Create an instance of the ImuDataProcessor class
    ImuData imu;

    // Create a subscriber for the IMU messages and specify the callback function
    ros::Subscriber sub = nh.subscribe("/turtlebot/kobuki/sensors/imu_data", 1000, &ImuData::imuCallback, &imu);

    // Spin
    ros::spin();

    return 0;
}
