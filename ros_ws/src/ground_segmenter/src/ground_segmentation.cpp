#include <chrono>
#include <memory>
#include <string>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "sensor_msgs/msg/point_cloud2.hpp"


class GroundSegmenter : public rclcpp::Node
{
public:
  GroundSegmenter()
  : Node("ground_segmenter")
  {
    auto topic_callback =
      [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) -> void {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = extract_points(*msg); // TODO: add an exception catch here
      };

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/lidar", rclcpp::SensorDataQoS(), topic_callback);
  }
  // Struct def for easier usage 
  struct Plane {float A; float B; float C; float D;}; 

private:
// Variable declarations
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

// Function declarations
// Extract points from PointCloud2 message
pcl::PointCloud<pcl::PointXYZ>::Ptr extract_points(const sensor_msgs::msg::PointCloud2 & msg)
  {
    auto cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(msg, *cloud);
    return cloud;
  }

// Computes the distance between a point and a plane
float distance_from_plane(const pcl::PointXYZ point, const Plane plane){
  float numerator = abs(plane.A * point.x + plane.B * point.y + plane.C * point.z + plane.D);
  float denominator = sqrt(pow(plane.A, 2) + pow(plane.C, 2) + pow(plane.B, 2)); // TODO: keep this elsewhere to reduce redundant computations
  float distance = numerator / denominator; 
  return distance;
}

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundSegmenter>());
  rclcpp::shutdown();
  return 0;
}