#ifndef SCAN_HANDLER_H
#define SCAN_HANDLER_H

#include "ros/ros.h"
#include <ros/package.h>
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
#include <tf/transform_broadcaster.h>
#include <Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <geometry_msgs/PoseArray.h>

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

    Eigen::Matrix4f transformation_inverse = transformation.inverse();

    return (transformation_inverse);
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

    Eigen::Matrix4f transformation_inverse = transformation.inverse();

    return (transformation_inverse);
}

class ScanHandler
{
public:
    ros::NodeHandle &nh_;
    ros::Publisher pointcloud_pub_ = this->nh_.advertise<sensor_msgs::PointCloud2>("/pc", 10);
    ros::Publisher keyframe_pub_ = this->nh_.advertise<geometry_msgs::PoseArray>("/keyframes", 10);

    // Laser Geometry Projection for converting scan to PointCloud
    laser_geometry::LaserProjection projector_;

    // Initialising listener transform
    tf::TransformListener listener_;

    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
    tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;

    tf::TransformBroadcaster keyframe_br_;

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

    Eigen::Matrix4f Twtok = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Tktokplus1 = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f Twtokplus1 = Eigen::Matrix4f::Identity();

    ScanHandler(ros::NodeHandle &nh, double thresholdTime, double thresholdOdometry) : nh_(nh), thresholdTime_(thresholdTime), thresholdOdometry_(thresholdOdometry), laser_sub_(nh, "/turtlebot/kobuki/sensors/rplidar", 1), laser_notifier_(laser_sub_, listener_, "turtlebot/kobuki/base_footprint", 1), client_(nh.serviceClient<turtlebot_graph_slam::ResetFilter>("ResetFilter"))
    {

        // Initialize subscribers
        odom_sub_ = nh.subscribe("/odom", 10, &ScanHandler::odometryCallback, this);

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

    void storeWorldPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        map_storedPointClouds_.push_back(cloud);
        std::string packagePath = ros::package::getPath("turtlebot_graph_slam");
        std::string directory = "/pcl_viz/";
        pcl::io::savePCDFileASCII(packagePath + directory + "world_map.pcd", *cloud);
    };

private:
    // A vector of Pointers to the PointClouds for storing Incoming scans
    // std::vector<pcl::PointCloud<pcl::PointXYZ>> storedPointClouds_;   // For directly storing pointclouds
    std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> storedPointClouds_;
    std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> map_storedPointClouds_;
    std::vector<tf::StampedTransform> storedKeyframes_;
    std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> hypothesis_;
    pcl::PointCloud<pcl::PointXYZ> world_map_;
    std::vector<Eigen::Matrix4f> keyframes_;
    // void publishMatching(const std::pair<double, double> &transformations, const std::vector<int> &matching_ids);

    // Scan Callback method
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
    {
        if (time_trigger_)
        {
            sensor_msgs::PointCloud2 cloud_in;
            try
            {
                ros::Time start_time_ = ros::Time::now();
                projector_.transformLaserScanToPointCloud("turtlebot/kobuki/base_footprint", *scan, cloud_in, listener_);

                last_scan_odom_ = current_odom_;
                odomtoTF(last_scan_odom_, current_key_frame);

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

                Eigen::Matrix4f current_tf = TFtoSE3(current_key_frame);

                std::cout << "Current Key Frame:\n"
                          << current_tf << std::endl;


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
                // pointcloud_pub_.publish(cloud_in);

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
                }
                else
                {
                    Twtok = TFtoSE3(current_key_frame);
                }
                ros::Time end_time_ = ros::Time::now();
                ros::Duration scan_processing_time = end_time_ - start_time_;

                ROS_INFO_STREAM(" Scan Processing Runtime: " << scan_processing_time.toSec());

                // publishMatching(Transformations_vector);

                // TODO: Registrations conditions
                if (true || current_scan_index == 0 || current_scan_index == 1 || current_scan_index == 2 || current_scan_index == 3)
                {
                    registerPointcloudinWorld(planarPointcloud);
                }

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
        // Saving current odometry
        current_odom_ = *odom;

        double dx = odom->pose.pose.position.x - (this->last_scan_odom_.pose.pose.position.x);
        double dy = odom->pose.pose.position.y - (this->last_scan_odom_.pose.pose.position.y);
        double displacement = sqrt(dx * dx + dy * dy);

        // Calculate the change in orientation
        double dtheta = atan2(dy, dx) - atan2(this->last_scan_odom_.pose.pose.orientation.y, this->last_scan_odom_.pose.pose.orientation.x);
        dtheta = fmod(dtheta + M_PI, 2 * M_PI) - M_PI; // Normalize to [-π, π]

        // Check if the change in orientation is greater than π/8 or the displacement is greater than the threshold
        if (fabs(dtheta) > M_PI / 8 || displacement > thresholdOdometry_)
        {
            odom_trigger_ = true;
        }
    };

    // Timer Callback
    void timerCallback(const ros::TimerEvent &)
    {
        // Check if enough time has passed since the last scan
        // if ((ros::Time::now() - lastScanTime_).toSec() > thresholdTime_)
        // {
        //     time_trigger_=true;
        // }
        time_trigger_ = true;
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
        hypothesis_.push_back(storedPointClouds_[storedPointClouds_.size() - 2]);
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

    void odomtoTF(const nav_msgs::Odometry &odom_in, tf::StampedTransform &tf_out)
    {
        // Extract position and orientation from the odometry message
        geometry_msgs::Pose pose = odom_in.pose.pose;
        tf::Quaternion q(pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        tf::Vector3 t(pose.position.x, pose.position.y, pose.position.z);

        // Create tf::Transform
        tf::Transform transform(q, t);

        // Invert the transform
        tf::Transform inverseTransform = transform.inverse();

        // Assign the inverted transform to tf_out
        tf_out = tf::StampedTransform(inverseTransform, odom_in.header.stamp, odom_in.header.frame_id, odom_in.child_frame_id);
    }

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
            icp.setMaximumIterations(5000);

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
            Tktokplus1 = icp.getFinalTransformation();

            if (icp.hasConverged())
            {
                double meanSquaredDistance = (double)icp.getFitnessScore();
                ROS_INFO("ICP Fitness score --- %f", meanSquaredDistance);

                // Plot Transformation saving condition
                if (current_scan_index == 2)
                {
                    savePointcloud(currentScan, hypothesis_[i], cloud_source_aligned);
                };

                if (meanSquaredDistance <= 2.0)
                {
                    // Create a TransformStamped message
                    std::string frameID = "jk";
                    std::string childID = "kj";
                    geometry_msgs::TransformStamped tfs = SE3toTF(Tktokplus1, frameID, childID);

                    // Add the transformation to the vector
                    transformations.push_back(tfs);
                };
            };
        };
        return transformations;
    };

    void savePointcloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &source, const pcl::PointCloud<pcl::PointXYZ>::Ptr &target, const pcl::PointCloud<pcl::PointXYZ>::Ptr &aligned)
    {
        // Get the path to the current ROS package
        std::string packagePath = ros::package::getPath("turtlebot_graph_slam");
        std::string directory = "/pcl_viz/";
        pcl::io::savePCDFileASCII(packagePath + directory + "source.pcd", *source);
        pcl::io::savePCDFileASCII(packagePath + directory + "target.pcd", *target);
        pcl::io::savePCDFileASCII(packagePath + directory + "aligned.pcd", *aligned);
    };

    void registerPointcloudinWorld(const pcl::PointCloud<pcl::PointXYZ>::Ptr &currentScan)
    {

        // std::cout << "T w to k for scan ID " << current_scan_index << ":\n"
        //           << Twtok << std::endl;
        // std::cout << "T k to k+1 for scan ID " << current_scan_index << ":\n"
        //           << Tktokplus1 << std::endl;

        Twtokplus1 = Twtok * Tktokplus1;

        keyframes_.push_back(Twtokplus1);

        // std::cout << "T w to k+1 for scan ID " << current_scan_index << ":\n"
        //           << Twtokplus1 << std::endl;

        publishKeyframeinMap();

        // Transform currentScan to map frame
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_world(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*currentScan, *transformed_cloud_world, Twtokplus1);

        // Store it in map_storedPointClouds_
        storeWorldPointCloud(transformed_cloud_world);

        // Combine the pointcloud
        if (world_map_.empty())
        {
            world_map_ = (*transformed_cloud_world);
        }
        else
        {

            world_map_ += (*transformed_cloud_world);
        };

        Twtok = Twtokplus1;

        // Publish the whole point cloud
        publishWorldPointcloud();
    };

    void publishWorldPointcloud()
    {
        sensor_msgs::PointCloud2 world_cloud_out;

        pcl::toROSMsg(world_map_, world_cloud_out);

        world_cloud_out.header.frame_id = "map";
        world_cloud_out.header.stamp = ros::Time::now();

        // Publish the point cloud
        pointcloud_pub_.publish(world_cloud_out);
    };

    void publishKeyframeinMap()
    {
        geometry_msgs::PoseArray poseArray;
        poseArray.header.frame_id = "map";
        poseArray.header.stamp = ros::Time::now();

        // Iterate over all keyframes
        for (size_t i = 0; i < keyframes_.size(); ++i)
        {
            geometry_msgs::Pose pose;
            pose.position.x = keyframes_[i](0, 3);
            pose.position.y = keyframes_[i](1, 3);
            pose.position.z = keyframes_[i](2, 3);

            Eigen::Quaternionf quaternion(keyframes_[i].block<3, 3>(0, 0));
            pose.orientation.x = quaternion.x();
            pose.orientation.y = quaternion.y();
            pose.orientation.z = quaternion.z();
            pose.orientation.w = quaternion.w();

            // Add the pose to the PoseArray
            poseArray.poses.push_back(pose);

            // // Publish the pose as a TF transform to visualize it as a frame in RViz
            // tf::Transform transform;
            // tf::poseMsgToTF(pose, transform);
            // tf::StampedTransform stampedTransform(transform, poseArray.header.stamp, "map", "keyframe_" + std::to_string(i));
            // keyframe_br_.sendTransform(stampedTransform);
        }

        keyframe_pub_.publish(poseArray);
    }
};

#endif // SCAN_HANDLER_H