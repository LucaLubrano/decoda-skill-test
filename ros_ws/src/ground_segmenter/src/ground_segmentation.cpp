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


// 3 dimensional cross product - hard coded for simplicity given we are constrained to three dimensional data
pcl::PointXYZ cross_product(pcl::PointXYZ vector_1, pcl::PointXYZ vector_2){
    // Initialise return vector
    pcl::PointXYZ return_vector;

    // Cross product
    return_vector.x = vector_1.y * vector_2.z - vector_1.z * vector_2.y;
    return_vector.y = vector_1.z * vector_2.x - vector_1.x * vector_2.z;
    return_vector.z = vector_1.x * vector_2.y - vector_1.y * vector_2.x;

    return return_vector;
}

pcl::PointXYZ vector_diff(pcl::PointXYZ vector_a, pcl::PointXYZ vector_b){
    // Initialise return vector
    pcl::PointXYZ vector_c;

    // compute difference
    vector_c.x = vector_a.x - vector_b.x;
    vector_c.y = vector_a.y - vector_b.y;
    vector_c.z = vector_a.z - vector_b.z;

    return vector_c;
}

// Solves the general solution to a plane from 3 vectors
Plane generate_plane(pcl::PointXYZ vector_a, pcl::PointXYZ vector_b, pcl::PointXYZ vector_c){
    Plane plane;
    
    // coplanar vectors
    pcl::PointXYZ coplanar_vector_a = vector_diff(vector_b, vector_a);
    pcl::PointXYZ coplanar_vector_b = vector_diff(vector_c, vector_a);

    std::cout << "edge1: " << coplanar_vector_a.x << "," << coplanar_vector_a.y << "," << coplanar_vector_a.z << std::endl;
    std::cout << "edge2: " << coplanar_vector_b.x << "," << coplanar_vector_b.y << "," << coplanar_vector_b.z << std::endl;

    // normal vector
    pcl::PointXYZ normal_vector = cross_product(coplanar_vector_a, coplanar_vector_b);

    // plane
    plane.A = normal_vector.x;
    plane.B = normal_vector.y;
    plane.C = normal_vector.z;
    plane.D = - (plane.A * vector_a.x) - (plane.B * vector_a.y) - (plane.C * vector_a.z);
    return plane;
}

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundSegmenter>());
  rclcpp::shutdown();
  return 0;
}