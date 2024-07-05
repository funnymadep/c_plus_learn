#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <iostream>

#include "pointT.h" // 替换为包含自定义点类型的实际头文件

typedef ouster_ros::OS1::PointOS1 PointT;  // 使用自定义点类型

int main(int argc, char** argv) {
    ros::init(argc, argv, "pcd_publisher");
    ros::NodeHandle nh;

    if (argc != 2) {
        ROS_ERROR("Need one and only one parameter for PCD file path.");
        return -1;
    }

    std::string pcd_file = argv[1];

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2>("/points", 1);

    std::cout << "111" << std::endl;
    pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);


    std::cout << "222" << std::endl;
    if (pcl::io::loadPCDFile<PointT>(pcd_file, *cloud) == -1) {
        PCL_ERROR("Couldn't read file %s\n", pcd_file.c_str());
        return -1;
    }

    std::cout << "333" << std::endl;
    // 对于缺少的字段赋默认值
    for (auto& point : cloud->points) {
        point.t = static_cast<uint32_t>(point.intensity);
        point.reflectivity = static_cast<uint16_t>(point.intensity);
        point.ring = 1;
        point.noise = 1;
        point.range = 1;
    }

    // for (auto& point : cloud->points) {
    //     std::cout << point.noise << " " << point.x << std::endl;
    // }

    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "map";

    std::cout << "444" << std::endl;

    ros::Rate loop_rate(1);
    while (ros::ok()) {
        output.header.stamp = ros::Time::now();
        pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
