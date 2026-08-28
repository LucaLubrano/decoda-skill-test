#include <chrono>
#include <memory>
#include <string>
#include <math.h>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "sensor_msgs/msg/point_cloud2.hpp"


#define RANSAC_THRESHOLD 2
#define RANSAC_SIMULATIONS 100

class GroundSegmenter : public rclcpp::Node
{
public:
  GroundSegmenter()
  : Node("ground_segmenter")
  {
    auto topic_callback =
      [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) -> void {
        
        // extract points and ransac
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = extract_points(*msg); // TODO: add an exception catch here
        Plane best_fit_plane = ransac(cloud);

        // loop variables
        pcl::PointXYZ pt;
        float pt_dist;
        pcl::PointCloud<pcl::PointXYZ>::Ptr inlier_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZ>::Ptr outlier_cloud(new pcl::PointCloud<pcl::PointXYZ>());
        
        // extract points that comply with plane of best fit
        for(size_t i = 0; i < cloud->points.size(); i++){
          pt = cloud->points[i];
          pt_dist = distance_from_plane(pt, best_fit_plane);

          if(pt_dist < RANSAC_THRESHOLD){
            inlier_cloud->points.push_back(pt);
          } else if (pt_dist >= RANSAC_THRESHOLD){
            outlier_cloud->points.push_back(pt);
          } else{
            RCLCPP_WARN(this->get_logger(), "Point not defined as an inlier or outlier");
          } 
        }

        inlier_publisher_->publish(convert_points(inlier_cloud));
        outlier_publisher_->publish(convert_points(outlier_cloud));
      };

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/lidar", rclcpp::SensorDataQoS(), topic_callback);
    inlier_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/ground_points", rclcpp::SensorDataQoS());
    outlier_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/obstacle_points", rclcpp::SensorDataQoS());

  }
  // Struct def for easier usage 
  struct Plane {float A; float B; float C; float D;}; 

private:
// Variable declarations
rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr inlier_publisher_;
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr outlier_publisher_;

// Function declarations
// Extract points from PointCloud2 message
pcl::PointCloud<pcl::PointXYZ>::Ptr extract_points(const sensor_msgs::msg::PointCloud2 & msg)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(msg, *cloud);
    return cloud;
  }

// Convert the cloud back to a PointCloud2 message
sensor_msgs::msg::PointCloud2 convert_points(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(*cloud, msg);
  return msg;
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
Plane generate_plane(std::vector<pcl::PointXYZ> vector_set){
    Plane plane;
    
    // coplanar vectors
    pcl::PointXYZ coplanar_vector_a = vector_diff(vector_set[1], vector_set[0]);
    pcl::PointXYZ coplanar_vector_b = vector_diff(vector_set[2], vector_set[0]);

    // normal vector
    pcl::PointXYZ normal_vector = cross_product(coplanar_vector_a, coplanar_vector_b);

    // plane
    plane.A = normal_vector.x;
    plane.B = normal_vector.y;
    plane.C = normal_vector.z;
    plane.D = - (plane.A * vector_set[0].x) - (plane.B * vector_set[0].y) - (plane.C * vector_set[0].z);
    return plane;
}

// Conduct the RANSAC algorithm
Plane ransac(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  const size_t total_points = cloud->points.size();

  // initialise loop variables
  Plane plane_of_best_fit;
  Plane proposed_plane;
  pcl::PointXYZ pt;
  int proposed_num_inliers = 0;
  int bestfit_num_inliers = 0;

  // iterate over a fixed number of simulations
  for(size_t i = 0; i < RANSAC_SIMULATIONS; i++){
    // generate a random plane
    std::vector<pcl::PointXYZ> rand_pt_set = sample_cloud(cloud);
    proposed_plane = generate_plane(rand_pt_set);
    
    // iterate over the entire cloud and compare to our proposed_plane
    for(size_t cur_pt_idx = 0; cur_pt_idx < total_points; cur_pt_idx++){
      // check the current distance from the point to the proposed plane
      pt = cloud->points[cur_pt_idx];
      float cur_pt_dist = distance_from_plane(pt, proposed_plane);

      // check if this distance is less than a prescribed threshold
      if(cur_pt_dist < RANSAC_THRESHOLD){
        proposed_num_inliers++;
      }
    }
    
    // Check if the proposed plane beats the current plane of best fit
    if(proposed_num_inliers > bestfit_num_inliers){
      plane_of_best_fit = proposed_plane;
      bestfit_num_inliers = proposed_num_inliers;
    }

    proposed_num_inliers = 0; // reset for next itr
  }

  return plane_of_best_fit;
}

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GroundSegmenter>());
  rclcpp::shutdown();
  return 0;
}