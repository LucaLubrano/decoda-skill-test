// Standard includes
#include <memory>
#include <chrono>

// ROS2 includes
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

// For timer
using namespace std::chrono_literals;

#define LIDAR_CUTOFF 1.0 // seconds

class LidarChecker : public rclcpp::Node
{
public:
  LidarChecker()
  : Node("lidar_checker")
  {
    last_received_ = this->now();

    auto topic_callback =
      [this](sensor_msgs::msg::PointCloud2::UniquePtr msg) -> void {
        last_received_ = this->now();
      };

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/lidar", rclcpp::SensorDataQoS(), topic_callback);
    timer_ = this->create_wall_timer(100ms, std::bind(&LidarChecker::check_health, this));
  }

private:
// Variable declarations
    rclcpp::Time last_received_; 
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

// Function declarations
    void check_health(){
        // Checks to see if we have a LiDAR signal incoming
        rclcpp::Duration time_passed = this->now() - last_received_;
        double elapsed_seconds = time_passed.seconds();
        if(elapsed_seconds > LIDAR_CUTOFF){
            RCLCPP_ERROR(this->get_logger(), "LiDAR data not received in %.2f", elapsed_seconds);
        } else {
          RCLCPP_INFO(this->get_logger(), "Receiving LiDAR data");
        }
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarChecker>());
  rclcpp::shutdown();
  return 0;
}