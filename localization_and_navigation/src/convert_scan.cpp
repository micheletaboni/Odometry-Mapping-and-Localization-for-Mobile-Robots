#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <bits/stdc++.h> //per massimo e minimo, funzione std::minmax_element
//per convert scan
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp" //poiche mi iscrivo a questo tipo per lo scan 
#include "sensor_msgs/msg/laser_scan.hpp" //poiche pubblico questo tipo di laser scan
#include "sensor_msgs/point_cloud2_iterator.hpp"


using std::placeholders::_1;


class ConvertScanNode : public rclcpp::Node
{
public:
  ConvertScanNode()
  : Node("convert_scan")
  {
    
    // TODO(hidmic): adjust default input queue size based on actual concurrency levels
    // achievable by the associated executor
    input_queue_size_ = this->declare_parameter(
    "queue_size", static_cast<int>(std::thread::hardware_concurrency()));
    min_height_ = this->declare_parameter("min_height", std::numeric_limits<double>::min());
    max_height_ = this->declare_parameter("max_height", std::numeric_limits<double>::max());
    angle_min_ = this->declare_parameter("angle_min", -M_PI);
    angle_max_ = this->declare_parameter("angle_max", M_PI);
    angle_increment_ = this->declare_parameter("angle_increment", M_PI / 180.0);
    scan_time_ = this->declare_parameter("scan_time", 1.0 / 30.0);
    range_min_ = this->declare_parameter("range_min", 0.0);
    range_max_ = this->declare_parameter("range_max", std::numeric_limits<double>::max());
    inf_epsilon_ = this->declare_parameter("inf_epsilon", 1.0);
    use_inf_ = this->declare_parameter("use_inf", true);

    publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("laser_scan", 10);
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "ugv/rslidar_points",
      10,
      std::bind(&ConvertScanNode::cloudCallback, this, _1));

    
    RCLCPP_INFO(this->get_logger(), "ConvertScanNode started.");
    RCLCPP_INFO(this->get_logger(), "Listening on /ugv/rslidar_points");
  }

private:

  double input_queue_size_, min_height_, max_height_, angle_min_, angle_max_, angle_increment_, scan_time_, range_min_, range_max_, inf_epsilon_;
  bool use_inf_;


  void cloudCallback(sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud_msg) //uso shared pointer e non & poiche sicuro ha una dimensione molto grande
  {
    
    // build laserscan output
    auto scan_msg = std::make_unique<sensor_msgs::msg::LaserScan>();
    scan_msg->header = cloud_msg->header;

    scan_msg->angle_min = angle_min_;
    scan_msg->angle_max = angle_max_;
    scan_msg->angle_increment = angle_increment_;
    scan_msg->time_increment = 0.0;
    scan_msg->scan_time = scan_time_;
    scan_msg->range_min = range_min_;
    scan_msg->range_max = range_max_;  
    
    // determine amount of rays to create
    uint32_t ranges_size = std::ceil(
     (scan_msg->angle_max - scan_msg->angle_min) / scan_msg->angle_increment);

    // determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
    if (use_inf_) {
    scan_msg->ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
    } else {
    scan_msg->ranges.assign(ranges_size, scan_msg->range_max + inf_epsilon_);
    }

    // Iterate through pointcloud
    for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_msg, "x"),
      iter_y(*cloud_msg, "y"), iter_z(*cloud_msg, "z");
      iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
    {
      if (std::isnan(*iter_x) || std::isnan(*iter_y) || std::isnan(*iter_z)) {
        RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for nan in point(%f, %f, %f)\n",
        *iter_x, *iter_y, *iter_z);
        continue;
      }

      if (*iter_z > max_height_ || *iter_z < min_height_) {
        RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for height %f not in range (%f, %f)\n",
        *iter_z, min_height_, max_height_);
        continue;
      }

      double range = hypot(*iter_x, *iter_y);
      if (range < range_min_) {
        RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for range %f below minimum value %f. Point: (%f, %f, %f)",
        range, range_min_, *iter_x, *iter_y, *iter_z);
        continue;
      }
      if (range > range_max_) {
        RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for range %f above maximum value %f. Point: (%f, %f, %f)",
        range, range_max_, *iter_x, *iter_y, *iter_z);
        continue;
      }

      double angle = atan2(*iter_y, *iter_x);
      if (angle < scan_msg->angle_min || angle > scan_msg->angle_max) {
        RCLCPP_DEBUG(
        this->get_logger(),
        "rejected for angle %f not in range (%f, %f)\n",
        angle, scan_msg->angle_min, scan_msg->angle_max);
        continue;
      }

    // overwrite range at laserscan ray if new range is smaller
      int index = (angle - scan_msg->angle_min) / scan_msg->angle_increment;
      if (range < scan_msg->ranges[index]) {
      scan_msg->ranges[index] = range;
      }
   }
    
    auto pair_max_min = std::minmax_element(scan_msg->ranges.begin(), scan_msg->ranges.end());
    RCLCPP_INFO(this->get_logger(),"arrivato %ld e i valore minimo %f, mentre il massimo %f", scan_msg->ranges.size(), *pair_max_min.first, *pair_max_min.second);

    publisher_->publish(std::move(scan_msg));

  }

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConvertScanNode>());
  rclcpp::shutdown();
  return 0;
}
