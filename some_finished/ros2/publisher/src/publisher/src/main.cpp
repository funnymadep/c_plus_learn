#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class PointCloudPublisher : public rclcpp::Node
{
public:
    PointCloudPublisher() : Node("point_cloud_publisher")
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("topic", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), std::bind(&PointCloudPublisher::timer_callback, this));
        load_point_cloud("/home/sd-robot/hql_test/model.pcd");
    }

private:
    void load_point_cloud(const std::string &file_path)
    {
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(file_path, *cloud_) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Couldn't read file %s", file_path.c_str());
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %d data points from %s", cloud_->width * cloud_->height, file_path.c_str());
    }

    void timer_callback()
    {
        sensor_msgs::msg::PointCloud2 output;
        pcl::toROSMsg(*cloud_, output);
        output.header.frame_id = "map";
        publisher_->publish(output);
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ {new pcl::PointCloud<pcl::PointXYZ>};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudPublisher>());
    rclcpp::shutdown();
    return 0;
}

