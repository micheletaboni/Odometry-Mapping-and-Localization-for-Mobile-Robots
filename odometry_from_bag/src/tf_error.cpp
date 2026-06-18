#include <algorithm>
#include <cctype>
#include <memory>
#include <string>
#include <cmath>
#include <chrono> //per il ritardino da aspettare
//per odometry
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "bunker_msgs/msg/bunker_status.hpp" //poiche mi iscrivo a questo tipo
#include "nav_msgs/msg/odometry.hpp" //poiche devo pubblicare questo tipo
#include <tf2/LinearMath/Quaternion.h> //per poter usare la trasformazione da angoli di eulero a quaternioni per odometry message
//per tf e ascoltare tf
#include "geometry_msgs/msg/transform_stamped.hpp" 
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h" //per try and catch fondamentale qua senno magari io posso avere errori
#include "tf2/LinearMath/Matrix3x3.h" //per convertire da quaternioni a theta
//per pubblicare nuovo custom message
#include "first_project_custom_msgs/msg/tf_error.hpp"



using std::placeholders::_1;
using std::placeholders::_2;

class tfError : public rclcpp::Node
{
public:
  tfError()
  : Node("tf_error")
  {

    //per ascoltare le informazioni dalle due tf
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    

    //per pubblicare custom message
     publisher_ = this->create_publisher<first_project_custom_msgs::msg::TfError>("tf_error_msg", 10);

    //mi iscrivo al mio topic /project_odom cosi riesco a calcolare ogni volta che arriva li un messaggio l'errore
     subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom",
      10,
      std::bind(&tfError::error, this, _1));

    RCLCPP_INFO(this->get_logger(), "tf_error started.");
    RCLCPP_INFO(this->get_logger(), "Listening on tf from base_link and base_link2");
    
  }

private:

  void error(const nav_msgs::msg::Odometry & msg)
  {
    
    geometry_msgs::msg::TransformStamped t; //odom - base_link
    geometry_msgs::msg::TransformStamped t2; //odom - base_link2

    try {
      t = tf_buffer_->lookupTransform("odom", "base_link", msg.header.stamp, std::chrono::milliseconds(50));
    } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Errore TF: %s | Tempo richiesto: %d.%d", 
                ex.what(), msg.header.stamp.sec, msg.header.stamp.nanosec);
      return;
    }

    try {
      t2 = tf_buffer_->lookupTransform("odom", "base_link2", msg.header.stamp, std::chrono::milliseconds(50));
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(this->get_logger(), "Lookup 2 failed: %s", ex.what());
      return;
    }

    //prima tf: odom - base_link
    const auto & r = t.transform.rotation;
    tf2::Quaternion q(r.x, r.y, r.z, r.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    //sevonda tf: odom -base_link2
    const auto & r2 = t2.transform.rotation;
    tf2::Quaternion q2(r2.x, r2.y, r2.z, r2.w);
    double roll2, pitch2, yaw2;
    tf2::Matrix3x3(q2).getRPY(roll2, pitch2, yaw2);


    //devo popolare il mio messaggio che pubblichero
    first_project_custom_msgs::msg::TfError msg_error;

    //header
    msg_error.header.stamp = msg.header.stamp; 
    //tempo trascorso
    const int time_now_sec = msg.header.stamp.sec;
    const int time_now_nanosec =  msg.header.stamp.nanosec;
    double time_now = time_now_sec + time_now_nanosec*1e-9;
    counter++;
    if(counter == 1)
    {
      time_prev = time_now;
      time_from_beginning = 0.0;
      msg_error.time_from_start = 0.0;
    }
    else
    {
      double dt = time_now - time_prev;
      time_prev = time_now;
      time_from_beginning = time_from_beginning + dt;
      msg_error.time_from_start = time_from_beginning;
    }
    
    //norm of the error:position
    msg_error.tf_error_position = std::sqrt(pow(t.transform.translation.x - t2.transform.translation.x, 2) +  pow(t.transform.translation.y - t2.transform.translation.y, 2));
    //norm of the error:orientation
    msg_error.tf_error_orientation = std::abs(yaw - yaw2);
    //distance travelled
    msg_error.travelled_distance =  std::sqrt(pow(t.transform.translation.x,2) + pow(t.transform.translation.y, 2));
    
    
    publisher_->publish(msg_error);
  }

  int counter = 0;
  double time_prev = 0;
  double time_from_beginning = 0;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Publisher<first_project_custom_msgs::msg::TfError>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tfError>());
  rclcpp::shutdown();
  return 0;
}
