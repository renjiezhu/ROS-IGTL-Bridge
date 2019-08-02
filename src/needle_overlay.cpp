/*
 * Software Interface for Image Guided Robotics
 * 
 * Needle overlay for 3D Slicer 
 * 
 * Topics Published:
 * * IGTL_TRANSFORM_OUT
 * 
 * Topics Subscribed:
 * * 
 * 
 * By Renjie Zhu (rezhu@eng.ucsd.edu)
 * 
 * July 3rd, 2019
 * 
 */

#include "ros/ros.h"
#include "geometry_msgs/Transform.h"

#include "ros_igtl_bridge/igtltransform.h"
#include "ros_igtl_bridge/igtlpointcloud.h"
#include <string>

class NeedleOverlay
{
public:
    NeedleOverlay(int argc, char *argv[], const char *node_name);
    ~NeedleOverlay();
    void Run() { ros::spin(); }

    void sendNeedleCallback(const geometry_msgs::TransformConstPtr &input);

private:
    ros::NodeHandle *nh;

    ros::Publisher needle_tr_pub;
    ros::Publisher needle_pc_pub;
    ros::Subscriber transform_sub;
};

NeedleOverlay::NeedleOverlay(int argc, char *argv[], const char *node_name)
{
    ros::init(argc, argv, node_name);
    nh = new ros::NodeHandle;

    needle_pc_pub = nh->advertise<ros_igtl_bridge::igtlpointcloud>("IGTL_POINTCLOUD_OUT", 2);
    needle_tr_pub = nh->advertise<ros_igtl_bridge::igtltransform>("IGTL_TRANSFORM_OUT", 10);
    transform_sub = nh->subscribe("robot_status_TRANSFORM", 1, &NeedleOverlay::sendNeedleCallback, this);
    // sendNeedle();
}

NeedleOverlay::~NeedleOverlay()
{
    delete nh;
}

void NeedleOverlay::sendNeedleCallback(const geometry_msgs::TransformConstPtr &input)
{
    ros_igtl_bridge::igtltransform transform_msg;
    transform_msg.name = "needle_pose";

    ROS_INFO("transform received: x=%.2f", input->translation.x);
    transform_msg.transform.translation.x = input->translation.x;
    transform_msg.transform.translation.y = input->translation.y;
    transform_msg.transform.translation.z = input->translation.z;

    transform_msg.transform.rotation.w = input->rotation.w;
    transform_msg.transform.rotation.x = input->rotation.x;
    transform_msg.transform.rotation.y = input->rotation.y;
    transform_msg.transform.rotation.z = input->rotation.z;

    // needle_tr_pub.publish(transform_msg);
    int npoints = 100;
    std::string points_name = "needle_pointcloud";
    ros_igtl_bridge::igtlpointcloud points_msg;
    points_msg.pointdata.resize(npoints);
    float max = 50.0;
    for (int i = 0; i < npoints; i++)
    {
        points_msg.pointdata[i].x = (float)rand() / (float)(RAND_MAX)*max;
        points_msg.pointdata[i].y = (float)rand() / (float)(RAND_MAX)*max;
        points_msg.pointdata[i].z = (float)rand() / (float)(RAND_MAX)*max;
    }
    points_msg.name = points_name;
    needle_pc_pub.publish(points_msg);
}

int main(int argc, char *argv[])
{
    NeedleOverlay test_needle(argc, argv, "ros_igtl_bridge_needle");
    test_needle.Run();

    return 0;
}
