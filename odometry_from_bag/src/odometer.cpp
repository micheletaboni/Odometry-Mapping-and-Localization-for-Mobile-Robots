#include <algorithm>
#include <cctype>
#include <memory>
#include <string>
#include <cmath>
//per odometry
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "bunker_msgs/msg/bunker_status.hpp" //poiche mi iscrivo a questo tipo
#include "nav_msgs/msg/odometry.hpp" //poiche devo pubblicare questo tipo
#include <tf2/LinearMath/Quaternion.h> //per poter usare la trasformazione da angoli di eulero a quaternioni per odometry message
//per tf
#include "geometry_msgs/msg/transform_stamped.hpp" 
#include "tf2_ros/transform_broadcaster.h"
//per custom service
#include "first_project/srv/reset_odometry.hpp"


using std::placeholders::_1;
using std::placeholders::_2;

class OdometerNode : public rclcpp::Node
{
public:
  OdometerNode()
  : Node("odometer")
  {
    //pubblica un messaggio di tipo nav_msgs::msg::Odometry sul topic /project_odom
    publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("project_odom", 10);

    //mi iscrivo al tipo ci custom message bunker_msgs::msg::BunkerStatus e al topic /bunker_status
    subscription_ = this->create_subscription<bunker_msgs::msg::BunkerStatus>(
      "bunker_status",
      10,
      std::bind(&OdometerNode::odometry, this, _1));

    //per tf, pubblica il messaggio
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    //per service reset_odometry
    service_ = this->create_service<first_project::srv::ResetOdometry>("reset_odometry", std::bind(&OdometerNode::odometry_reset, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "OdometerNode started.");
    RCLCPP_INFO(this->get_logger(), "Listening on /bunker_status");
    RCLCPP_INFO(this->get_logger(), "Publishing odometry of the robot and TF odom-base_link2");
  }

private:
  void odometry_reset(const std::shared_ptr<first_project::srv::ResetOdometry::Request> request,
    std::shared_ptr<first_project::srv::ResetOdometry::Response> response)
    {
      RCLCPP_INFO(this->get_logger(), "Request received: Reset Odometry");
      x_prev = 0;
      y_prev = 0;
      z_prev = 0;
      theta_prev = 0;
      counter = 0;
      (void)request;
      (void)response;      
    
    }

  void odometry(const bunker_msgs::msg::BunkerStatus & msg)
  {
    
    counter++;
    double v_R = 0.0;
    double v_L = 0.0;

    if (msg.actuator_states[0].motor_id == msg.MOTOR_ID_FRONT_RIGHT && msg.actuator_states[0].driver_temperature != 0) //RIGHT, filtro segnale non fasullo attraverso temperatura sempre diversa da zero
    {
      v_R = radius*(msg.actuator_states[0].rpm *2*M_PI)/60;
    }

    if (msg.actuator_states[1].motor_id == msg.MOTOR_ID_FRONT_LEFT) //LEFT
    {
      v_L = radius*(msg.actuator_states[1].rpm *2*M_PI)/60;
    }

    //calcolate da rpm motori
    float linear_velocity_now = (v_L + v_R)/2;
    float angular_velocity_now = (v_R - v_L)/L;

    //reali
    /*float linear_velocity_now1 = msg.linear_velocity;*/
    float angular_velocity_now1 = msg.angular_velocity;
    
    const int time_now_sec = msg.header.stamp.sec;
    const int time_now_nanosec =  msg.header.stamp.nanosec;
    double time_now = time_now_sec + time_now_nanosec*1e-9;

    //compute the values to put them in the odometry message
    nav_msgs::msg::Odometry odometry_values;
    // 1. Copia l'orario esatto in cui il bunker ha acquisito il dato
    odometry_values.header.stamp = msg.header.stamp;

    // 2. Dichiara il sistema di riferimento fisso (padre)
    odometry_values.header.frame_id = "odom";

    // 3. Dichiara il sistema di riferimento mobile del robot (figlio)
    odometry_values.child_frame_id = "base_link2";
    float theta;
    tf2::Quaternion q;

    if (counter == 1) //assumo partano da zero!!!
    {
      //position
      odometry_values.pose.pose.position.x = 0;
      odometry_values.pose.pose.position.y = 0;
      odometry_values.pose.pose.position.z = 0;
      
      x_prev = odometry_values.pose.pose.position.x;
      y_prev = odometry_values.pose.pose.position.y;
      z_prev = odometry_values.pose.pose.position.z;
      
      //orientation
      theta = 0;
      theta_prev = theta;
      q.setRPY(0.0, 0.0, theta);
      odometry_values.pose.pose.orientation.x = q.x();
      odometry_values.pose.pose.orientation.y = q.y();
      odometry_values.pose.pose.orientation.z = q.z();
      odometry_values.pose.pose.orientation.w = q.w();

      time_prev = time_now;

      publisher_->publish(odometry_values); //pubblico il messaggio

    }
    else if(counter != 1 && std::abs(angular_velocity_now) < 1e-3) //use runge-kutta in case w too small
    {
      double dt = time_now - time_prev;
      time_prev = time_now;
      //orientation
      theta = theta_prev + angular_velocity_now*dt;
      
      q.setRPY(0.0, 0.0, theta);
      odometry_values.pose.pose.orientation.x = q.x();
      odometry_values.pose.pose.orientation.y = q.y();
      odometry_values.pose.pose.orientation.z = q.z();
      odometry_values.pose.pose.orientation.w = q.w();

      //position
      odometry_values.pose.pose.position.x = x_prev + linear_velocity_now*dt*cos(theta_prev + (angular_velocity_now*dt)/2);
      odometry_values.pose.pose.position.y = y_prev + linear_velocity_now*dt*sin(theta_prev + (angular_velocity_now*dt)/2);
      odometry_values.pose.pose.position.z = 0;

      theta_prev = theta;
      x_prev = odometry_values.pose.pose.position.x;
      y_prev = odometry_values.pose.pose.position.y;
      z_prev = odometry_values.pose.pose.position.z;

      publisher_->publish(odometry_values); //pubblico il messaggio

    }
    else if (counter!=1 && std::abs(angular_velocity_now) > 1e-5) //uso integrazione esatta se la velocita angolare non è troppo bassa
    {
      double dt = time_now - time_prev;
      time_prev = time_now;
      //orientation
      theta = theta_prev + angular_velocity_now*dt;
      
      q.setRPY(0.0, 0.0, theta);
      odometry_values.pose.pose.orientation.x = q.x();
      odometry_values.pose.pose.orientation.y = q.y();
      odometry_values.pose.pose.orientation.z = q.z();
      odometry_values.pose.pose.orientation.w = q.w();

      //position
      odometry_values.pose.pose.position.x = x_prev + (linear_velocity_now/angular_velocity_now)*(sin(theta) - sin(theta_prev));
      odometry_values.pose.pose.position.y = y_prev - (linear_velocity_now/angular_velocity_now)*(cos(theta) - cos(theta_prev));
      odometry_values.pose.pose.position.z = 0;

      theta_prev = theta;
      x_prev = odometry_values.pose.pose.position.x;
      y_prev = odometry_values.pose.pose.position.y;
      z_prev = odometry_values.pose.pose.position.z;

      publisher_->publish(odometry_values); //pubblico il messaggio

    }

    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = msg.header.stamp;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link2";

    //position
    t.transform.translation.x = x_prev;
    t.transform.translation.y = y_prev;
    t.transform.translation.z = 0.0;
    //orientation
    q.setRPY(0.0, 0.0, theta);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_broadcaster_->sendTransform(t);

  }

  const double radius = 0.0112; //calcolato a mano, vedi appunti ipad
  const double L = 0.705; //da dati slide e ridotto da centro cingolo a centro cingolo e fine tuning su simulazioni
  double x_prev;
  double y_prev;
  double z_prev;
  double theta_prev = 0;
  int counter = 0;
  double dt = 0;
  double time_prev = 0;

  rclcpp::Service<first_project::srv::ResetOdometry>::SharedPtr service_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
  rclcpp::Subscription<bunker_msgs::msg::BunkerStatus>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdometerNode>());
  rclcpp::shutdown();
  return 0;
}
