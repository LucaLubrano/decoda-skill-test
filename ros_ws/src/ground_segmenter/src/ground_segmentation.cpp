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

// Samples the point cloud, used to generate plane for RANSAC process
std::vector<pcl::PointXYZ> sample_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    // TODO: put in loop outside this function
    // Prepare random device
    std::random_device rd;
    std::mt19937 gen(rd());

    // Determine bounds of RNG
    std::uniform_int_distribution<int> unif(0, cloud->points.size()-1); 

    // Initialise output to zero
    pcl::PointXYZ zero_point(0.0f, 0.0f, 0.0f); // enforces zero on initialisation
    std::vector<pcl::PointXYZ> point_set(3, zero_point);

    bool loop_flag = true;

    int randn;
    int randn_1 = -1;
    int randn_2 = -1;

    for(size_t i = 0; i < 3; i++){
        // Sample the cloud
        randn = unif(gen);
        
        /*
        Ensure that the indexes are not the same as 
        using the same point will defy the assumption
        that the points are not collinear in the plane
        construction step
        */    
        while((randn == randn_1) || (randn == randn_2)){
            randn = unif(gen);
        }

        point_set[i] = cloud->points[randn];
        
        if(i == 0){randn_1 = randn;}
        if(i == 1){randn_2 = randn;}
    }
    return point_set;
}

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundSegmenter>());
  rclcpp::shutdown();
  return 0;
}