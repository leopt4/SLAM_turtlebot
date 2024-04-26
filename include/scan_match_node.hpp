#ifndef SCAN_HANDLER_H
#define SCAN_HANDLER_H

#include "ros/ros.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h> // Include tf header
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <pcl/registration/icp.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_datatypes.h>
#include <Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>

#include "turtlebot_graph_slam/ResetFilter.h"

#include <sstream>

#include <iostream>
#include <vector>

// using namespace std;

template <typename T>
Eigen::Matrix4f TFtoSE3(const T &transform);

// Specialization for geometry_msgs::TransformStamped
template <>
Eigen::Matrix4f TFtoSE3<geometry_msgs::TransformStamped>(const geometry_msgs::TransformStamped &transformStamped)
{
    const geometry_msgs::Quaternion &q = transformStamped.transform.rotation;
    const geometry_msgs::Vector3 &t = transformStamped.transform.translation;

    Eigen::Quaternionf quaternion(q.w, q.x, q.y, q.z);
    Eigen::Vector3f translation(t.x, t.y, t.z);

    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    transformation.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
    transformation.block<3, 1>(0, 3) = translation;

    return (transformation.inverse());
}

// Specialization for tf::StampedTransform
template <>
Eigen::Matrix4f TFtoSE3<tf::StampedTransform>(const tf::StampedTransform &stampedTransform)
{
    const tf::Quaternion &q = stampedTransform.getRotation();
    const tf::Vector3 &t = stampedTransform.getOrigin();

    Eigen::Quaternionf quaternion(q.w(), q.x(), q.y(), q.z());
    Eigen::Vector3f translation(t.x(), t.y(), t.z());

    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    transformation.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
    transformation.block<3, 1>(0, 3) = translation;

    return (transformation.inverse());
}

class ScanHandler
{
public:
    ros::NodeHandle &nh_;
    ros::Publisher demo_ = this->nh_.advertise<sensor_msgs::PointCloud2>("/pc", 1);

    // Laser Geometry Projection for converting scan to PointCloud
    laser_geometry::LaserProjection projector_;

    // Initialising listener transform
    tf::TransformListener listener_;

    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;

    double thresholdTime_;
    double thresholdOdometry_;
    ros::Subscriber odom_sub_;
    ros::Timer timer_;

    bool odom_trigger_ = true;
    bool time_trigger_ = true;
    ros::Time lastScanTime_;
    nav_msgs::Odometry last_scan_odom_;
    nav_msgs::Odometry current_odom_;
    int current_scan_index = -1;
    tf::StampedTransform current_key_frame;
    ros::ServiceClient client_;

    ScanHandler(ros::NodeHandle &nh, double thresholdTime, double thresholdOdometry) : nh_(nh), thresholdTime_(thresholdTime), thresholdOdometry_(thresholdOdometry), laser_sub_(nh, "/turtlebot/kobuki/sensors/rplidar", 1), laser_notifier_(laser_sub_, listener_, "turtlebot/kobuki/predicted_base_footprint", 1), client_(nh.serviceClient<turtlebot_graph_slam::ResetFilter>("ResetFilter"))
    {

        // Initialize subscribers
        // odom_sub_ = nh.subscribe("/turtlebot/kobuki/odom_ground_truth", 1, &ScanHandler::odometryCallback, this);

        // Timer for getting a new scan
        timer_ = nh.createTimer(ros::Duration(thresholdTime_), &ScanHandler::timerCallback, this);

        // Laser notifiers for the scans
        laser_notifier_.registerCallback(boost::bind(&ScanHandler::scanCallback, this, _1));
        laser_notifier_.setTolerance(ros::Duration(0.01));
        lastScanTime_ = ros::Time::now();
    };

    void storePointCloudandKeyframe(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, const tf::StampedTransform &keyframe)
    {
        // std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudPtr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cloud);
        // storedPointClouds_.push_back(cloudPtr);
        storedPointClouds_.push_back(cloud);
        storedKeyframes_.push_back(keyframe);
    };

private:
    // A vector of Pointers to the PointClouds for storing Incoming scans
    // std::vector<pcl::PointCloud<pcl::PointXYZ>> storedPointClouds_;   // For directly storing pointclouds
    std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> storedPointClouds_;
    std::vector<tf::StampedTransform> storedKeyframes_;
    std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> hypothesis_;
    // void publishMatching(const std::pair<double, double> &transformations, const std::vector<int> &matching_ids);

    // Scan Callback method
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
    {
        if (time_trigger_)
        {
            sensor_msgs::PointCloud2 cloud_in;
            try
            {
                // Saving the current key frame of the robot
                std::string targetFrame = "turtlebot/kobuki/predicted_base_footprint";
                std::string sourceFrame = "world_ned";
                ros::Time scanTime = scan->header.stamp;

                // Wait for the transformation to become available
                listener_.waitForTransform(targetFrame, sourceFrame, scanTime, ros::Duration(0.1));

                // Now, attempt to look up the transformation
                listener_.lookupTransform(targetFrame, sourceFrame, scanTime, current_key_frame);

                projector_.transformLaserScanToPointCloud("turtlebot/kobuki/predicted_base_footprint", *scan, cloud_in, listener_);

                // Convert to PCL format for processing
                pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::fromROSMsg(cloud_in, *pclCloud);

                // Only taking the planar PointCloud model
                pcl::PointCloud<pcl::PointXYZ>::Ptr planarPointcloud = fitPlanarModel(pclCloud);

                // Storing the Pointcloud and keyframe
                storePointCloudandKeyframe(planarPointcloud, current_key_frame); // Pass the PCL point cloud pointer
                current_scan_index++;

                ROS_INFO("---Pointcloud (ID - %d) Received and Stored---", current_scan_index);

                // Publishing Obtained Pointcloud
                demo_.publish(cloud_in);

                if (current_scan_index > 0)
                {
                    // Hypothesis generation and matching
                    pclMatchHypothesis();
                    std::vector<geometry_msgs::TransformStamped> Transformations_vector = pointCloudMatching(planarPointcloud);
                    ROS_INFO("---Factors for Pointcloud (ID - %d) Calculated---", current_scan_index);
                    // ROS_INFO("SIZE of the transformations_vector -- %zu ", Transformations_vector.size());
                    for (const auto &transform : Transformations_vector)
                    {
                        double theta = tf::getYaw(transform.transform.rotation);
                        ROS_INFO("Transformation : (%f,%f,%f)",
                                 transform.transform.translation.x,
                                 transform.transform.translation.y,
                                 theta);
                    };
                };

                // publishMatching(Transformations_vector);
                last_scan_odom_ = current_odom_;

                turtlebot_graph_slam::ResetFilter srv;
                srv.request.reset_filter_requested = true;

                if (client_.call(srv))
                {
                    ROS_INFO("Odometry Filter (EKF) is succesfully reset.... : %d", (bool)srv.response.reset_filter_response);
                }
                else
                {
                    ROS_ERROR("Failed to call reset Odometry Filter!!!!!!!");
                };

                time_trigger_ = false;
            }
            catch (tf::TransformException &e)
            {
                ROS_ERROR("Transform error: %s", e.what());
                return;
            };
        }
        else
        {
            return;
        };
    };

    // Odometry Callback
    void odometryCallback(const nav_msgs::Odometry::ConstPtr &odom)
    {
        // Saving current odom
        current_odom_ = *odom;

        double dx = odom->pose.pose.position.x - (this->last_scan_odom_.pose.pose.position.x);
        double dy = odom->pose.pose.position.y - (this->last_scan_odom_.pose.pose.position.y);
        double displacement = sqrt(dx * dx + dy * dy);

        if (displacement > (thresholdOdometry_))
        {
            odom_trigger_ = true;
        };
    };

    // Timer Callback
    void timerCallback(const ros::TimerEvent &)
    {
        // Check if enough time has passed since the last scan
        // if ((ros::Time::now() - lastScanTime_).toSec() > thresholdTime_)
        // {
        //     time_trigger_=true;
        // }
        this->time_trigger_ = true;
    };

    pcl::PointCloud<pcl::PointXYZ>::Ptr fitPlanarModel(pcl::PointCloud<pcl::PointXYZ>::Ptr &currentScan)
    {

        // Check if the points are planar
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(0.01);
        seg.setInputCloud(currentScan);
        seg.segment(*inliers, *coefficients);
        if (inliers->indices.size() == 0)
        {
            // No planar model found
            ROS_INFO("NO Planar model found for the current scan !!");
        }

        // Extract only the inliers from the point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr inlierCloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(currentScan);
        extract.setIndices(inliers);
        extract.setNegative(false); // Set to true to get outliers
        extract.filter(*inlierCloud);

        return inlierCloud;
    };

    void pclMatchHypothesis()
    {
        hypothesis_.clear();
        hypothesis_.push_back(*(storedPointClouds_.end() - 2));
    };

    geometry_msgs::TransformStamped SE3toTF(const Eigen::Matrix4f &transformation, const std::string &frame_id, const std::string &child_frame_id)
    {
        // Extract the rotation matrix and translation vector from the transformation matrix
        Eigen::Matrix3f rotationMatrix = transformation.block<3, 3>(0, 0);
        Eigen::Vector3f translationVector = transformation.block<3, 1>(0, 3);

        // Convert the rotation matrix to a quaternion
        Eigen::Quaternionf quaternion(rotationMatrix);

        // Construct the Transform message
        geometry_msgs::Transform transform;
        transform.translation.x = translationVector.x();
        transform.translation.y = translationVector.y();
        transform.translation.z = translationVector.z();
        transform.rotation.x = quaternion.x();
        transform.rotation.y = quaternion.y();
        transform.rotation.z = quaternion.z();
        transform.rotation.w = quaternion.w();

        // Create a TransformStamped message
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.transform = transform;
        transformStamped.header.frame_id = frame_id;
        transformStamped.child_frame_id = child_frame_id;
        transformStamped.header.stamp = ros::Time::now();

        return transformStamped;
    };

    std::vector<geometry_msgs::TransformStamped> pointCloudMatching(const pcl::PointCloud<pcl::PointXYZ>::Ptr &currentScan)
    {
        std::vector<geometry_msgs::TransformStamped> transformations;

        for (size_t i = 0; i < hypothesis_.size(); ++i)
        {
            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setInputSource(currentScan);
            icp.setInputTarget(hypothesis_[i]);

            // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
            icp.setMaxCorrespondenceDistance(0.05);

            // Set the maximum number of iterations
            // icp.setMaximumIterations(100);

            // Set the transformation epsilon
            icp.setTransformationEpsilon(1e-6);

            // Set the euclidean distance difference epsilon
            icp.setEuclideanFitnessEpsilon(0.05);

            // Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
            Eigen::Matrix4f initial_guess = TFtoSE3(current_key_frame);
            std::cout << "Initial guess matrix:\n"
                      << initial_guess << std::endl;

            // Create an output point cloud for the aligned source
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source_aligned(new pcl::PointCloud<pcl::PointXYZ>);

            // Compute the transformation
            icp.align(*cloud_source_aligned, initial_guess);

            // Obtain the final transformation matrix
            Eigen::Matrix4f final_transformation = icp.getFinalTransformation();

            if (icp.hasConverged())
            {
                double meanSquaredDistance = (double)icp.getFitnessScore();
                ROS_INFO("ICP Fitness score --- %f", meanSquaredDistance);
                // Plot Transformation saving condition
                if (current_scan_index == 1)
                {
                    savePointcloud(currentScan, hypothesis_[i], cloud_source_aligned);
                };

                if (meanSquaredDistance <= 1.0)
                {
                    // Create a TransformStamped message
                    std::string frameID="jk";
                    std::string childID="kj";
                    geometry_msgs::TransformStamped tfs = SE3toTF(final_transformation, frameID, childID);

                    // Add the transformation to the vector
                    transformations.push_back(tfs);
                };
            };
        };
        return transformations;
    };

    void savePointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source, const pcl::PointCloud<pcl::PointXYZ>::Ptr &target, const pcl::PointCloud<pcl::PointXYZ>::Ptr &aligned)
    {
        // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        // viewer->setBackgroundColor(0, 0, 0);
        // std::string frameId = "Current_Frame_ID-" + std::to_string(current_scan_index);
        // std::string frameIdAligned = "Current_Frame_ID-" + std::to_string(current_scan_index) + "_Aligned";
        // std::string targetId = "Target_Frame_ID-" + std::to_string(current_scan_index - 1);
        // viewer->addPointCloud<pcl::PointXYZ>(target, targetId);
        // viewer->addPointCloud<pcl::PointXYZ>(source, frameId);
        // viewer->addPointCloud<pcl::PointXYZ>(aligned, frameIdAligned);
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, targetId);
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, frameId);
        // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, frameIdAligned);
        // std::string file_name = "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/pcl_viz/" + frameId + ".png";
        // viewer->saveScreenshot(file_name);

        std::string directory = "/home/patweatharva/ros_work/turtlebot_class/src/turtlebot_graph_slam/pcl_viz/";
        pcl::io::savePCDFileASCII(directory + "source.pcd", *source);
        pcl::io::savePCDFileASCII(directory + "target.pcd", *target);
        pcl::io::savePCDFileASCII(directory + "aligned.pcd", *aligned);
    };
};

#endif // SCAN_HANDLER_H