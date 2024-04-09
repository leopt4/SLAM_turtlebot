// // Include the ROS C++ APIs
// #include <ros/ros.h>
// // Standard C++ entry point
// int main(int argc, char** argv) {
//     // Announce this program to the ROS master as a "node" called "hello_world_node"
//     ros::init(argc, argv, "hello_world_node");
//     // Start the node resource managers (communication, time, etc)
//     ros::start();
//     // Broadcast a simple log message
//     ROS_INFO_STREAM("Hello, world!");
//     // Process ROS callbacks until receiving a SIGINT (ctrl-c)
//     ros::spin();
//     // Stop the node's resources
//     ros::shutdown();
//     // Exit tranquilly
//     return 0;
// }


#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "Publisher");
    ros::NodeHandle nh;
    ros::Publisher topic_pub = nh.advertise<std_msgs::String>("turtlebot_graph_slam", 1000);
    ros::Rate loop_rate(1);
    while(ros::ok()) {
        std_msgs::String msg; 
        msg.data = "Hello World!";
        topic_pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}