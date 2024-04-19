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
#include <sstream>

#include <iostream>
#include <vector>

// using namespace std;

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

    ScanHandler(ros::NodeHandle &nh, double thresholdTime, double thresholdOdometry) : nh_(nh), thresholdTime_(thresholdTime), thresholdOdometry_(thresholdOdometry), laser_sub_(nh, "/turtlebot/kobuki/sensors/rplidar", 1), laser_notifier_(laser_sub_, listener_, "turtlebot/kobuki/base_footprint", 1)
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

    void storePointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud)
    {
        // std::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> cloudPtr = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cloud);
        // storedPointClouds_.push_back(cloudPtr);
        storedPointClouds_.push_back(cloud);
    };

private:
    // A vector of Pointers to the PointClouds for storing Incoming scans
    // std::vector<pcl::PointCloud<pcl::PointXYZ>> storedPointClouds_;   // For directly storing pointclouds
    std::vector<boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>>> storedPointClouds_;

    // void publishMatching(const std::pair<double, double> &transformations, const std::vector<int> &matching_ids);

    // Scan Callback method
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
    {
        if (this->time_trigger_)
        {
            sensor_msgs::PointCloud2 cloud_in;
            try
            {
                projector_.transformLaserScanToPointCloud("turtlebot/kobuki/base_footprint", *scan, cloud_in, listener_);

                // Convert to PCL format for processing
                pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud(new pcl::PointCloud<pcl::PointXYZ>);
                pcl::fromROSMsg(cloud_in, *pclCloud);
                this->storePointCloud(pclCloud); // Pass the PCL point cloud pointer
                this->current_scan_index++;

                ROS_INFO("---Pointcloud (ID - %zu) Received and Stored---", ((this->storedPointClouds_.size()) - 1));

                // Publishing Obtained Pointcloud
                this->demo_.publish(cloud_in);

                // Hypothesis generation and matching
                // // std::vector<geometry_msgs::TransformStamped> Transformations_vector = pointCloudMatchHypothesis(pclCloud);
                // ROS_INFO("---Factors for Pointcloud (ID - %zu) Calculated---", ((this->storedPointClouds_.size()) - 1));
                // ROS_INFO("SIZE of the transformations_vector -- %zu ", Transformations_vector.size());
                // for (const auto &transform : Transformations_vector)
                // {
                //     double theta = tf::getYaw(transform.transform.rotation);
                //     ROS_INFO("Transformation : (%f,%f,%f)",
                //              transform.transform.translation.x,
                //              transform.transform.translation.y,
                //              theta);
                // };

                // publishMatching(Transformations_vector);

                this->time_trigger_ = false;
                this->last_scan_odom_ = current_odom_;
            }
            catch (tf::TransformException &e)
            {
                ROS_ERROR("Transform error: %s", e.what());
                return;
            }
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
        this->current_odom_ = *odom;

        double dx = odom->pose.pose.position.x - (this->last_scan_odom_.pose.pose.position.x);
        double dy = odom->pose.pose.position.y - (this->last_scan_odom_.pose.pose.position.y);
        double displacement = sqrt(dx * dx + dy * dy);

        if (displacement > (this->thresholdOdometry_))
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

    std::vector<geometry_msgs::TransformStamped> pointCloudMatchHypothesis(const pcl::PointCloud<pcl::PointXYZ>::Ptr &currentScan)
    {
        std::vector<geometry_msgs::TransformStamped> transformations;

        // pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        // viewer->setBackgroundColor(0, 0, 0);

        for (size_t i = 0; i < storedPointClouds_.size(); ++i)
        {
            // Check if the points are planar
            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.01);
            seg.setInputCloud(storedPointClouds_[i]);
            seg.segment(*inliers, *coefficients);

            if (inliers->indices.size() == 0)
            {
                // No planar model found, skip this scan
                continue;
                ROS_INFO("NO Planar model found");
            }

            pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
            icp.setInputSource(currentScan);
            icp.setInputTarget(storedPointClouds_[i]);
            pcl::PointCloud<pcl::PointXYZ> alignedCloud;
            icp.align(alignedCloud);

            if (icp.hasConverged())
            {
                double meanSquaredDistance = (double)icp.getFitnessScore();
                ROS_INFO("ICP Fitness score --- %f", meanSquaredDistance);
                // Plot Transformation saving condition
                // if ()
                // {
                //     std::string frameId = "Current_Frame_ID - " + std::to_string(current_scan_index);
                //     std::string frameIdAligned = "Current_Frame_ID - " + std::to_string(current_scan_index) + "_Aligned";
                //     std::string targetId = "Target_Frame_ID - " + std::to_string(i);
                //     viewer->addPointCloud<pcl::PointXYZ>(storedPointClouds_[i], targetId);
                //     viewer->addPointCloud<pcl::PointXYZ>(currentScan, frameId);
                //     pcl::PointCloud<pcl::PointXYZ>::Ptr alignedCloudPtr(new pcl::PointCloud<pcl::PointXYZ>(alignedCloud));
                //     viewer->addPointCloud<pcl::PointXYZ>(alignedCloudPtr, frameIdAligned);
                //     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, targetId);
                //     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, frameId);
                //     viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, frameIdAligned);
                //     std::string file_name = "pcl_viz/"+frameId+".png";
                //     viewer->saveScreenshot(file_name);
                // };

                if (meanSquaredDistance <= 0.8)
                {
                    // Extract the transformation matrix from current scan to store scan
                    Eigen::Matrix4f transformationMatrix_c2s = icp.getFinalTransformation();

                    // Invert the transformation matrix to get the transformation from the stored scan to the input scan
                    Eigen::Matrix4f transformationMatrix = transformationMatrix_c2s.inverse();

                    // Extract translation (x, y) and rotation (theta)
                    double x = transformationMatrix(0, 3) * 10;
                    double y = transformationMatrix(1, 3) * 10;
                    double theta = tf::getYaw(tf::Quaternion(
                        transformationMatrix(0, 0),
                        transformationMatrix(1, 0),
                        transformationMatrix(2, 0),
                        transformationMatrix(3, 0)));

                    // Create a TransformStamped message
                    geometry_msgs::TransformStamped transformStamped;
                    transformStamped.header.stamp = ros::Time::now();
                    transformStamped.header.frame_id = "world";                    // Assuming "world" is the fixed frame
                    transformStamped.child_frame_id = "scan_" + std::to_string(i); // Unique child frame for each scan
                    transformStamped.transform.translation.x = x;
                    transformStamped.transform.translation.y = y;
                    transformStamped.transform.translation.z = 0.0; // Assuming the robot moves in the XY plane
                    transformStamped.transform.rotation.x = 0.0;
                    transformStamped.transform.rotation.y = 0.0;
                    transformStamped.transform.rotation.z = sin(theta / 2.0);
                    transformStamped.transform.rotation.w = cos(theta / 2.0);

                    // Add the transformation to the vector
                    transformations.push_back(transformStamped);

                    // Publish the transformation
                    // transform_pub_.publish(transformStamped);
                };
            };
        };
        return transformations;
    };
};

#endif // SCAN_HANDLER_H