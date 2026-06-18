#include <chrono>
#include <cstdlib>
#include <functional>
#include <future>
#include <memory>
#include <string>
#include <vector>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <tf2/LinearMath/Quaternion.h> 
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp> //per il folder csv
#include <geometry_msgs/msg/pose_stamped.hpp>

using namespace std::chrono_literals;

class PoseClient : public rclcpp::Node
{
public:

  PoseClient()
  : Node("poses_client")
  {
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateThroughPoses>(this, "navigate_through_poses");
  }

  void send_goal(std::vector<std::vector<double>> & csv_values)
  {
    nav2_msgs::action::NavigateThroughPoses::Goal goal_msg;
    
    std::vector<double> row;
    tf2::Quaternion q;

    for(size_t i = 0; i < csv_values.size(); i++)
    {
        row = csv_values[i];
        geometry_msgs::msg::PoseStamped goal_temporaneo;
        goal_temporaneo.header.frame_id = "map";
        goal_temporaneo.header.stamp = this->now();
        goal_temporaneo.pose.position.x = row[0];
        goal_temporaneo.pose.position.y = row[1];

        q.setRPY(0.0, 0.0, row[2]);
        goal_temporaneo.pose.orientation.x = q.x();
        goal_temporaneo.pose.orientation.y = q.y();
        goal_temporaneo.pose.orientation.z = q.z();
        goal_temporaneo.pose.orientation.w = q.w();

        RCLCPP_INFO(this->get_logger(), "Posa letta e inviata: x=%.2f, y=%.2f, theta=%.2f", row[0], row[1], row[2]);
        goal_msg.poses.push_back(goal_temporaneo);      

    }

    

    RCLCPP_INFO(this->get_logger(), "In attesa dell'Action Server 'navigate_through_poses'...");
    
    // RIMETTI QUESTA PARTE: Attende max 10 secondi per connettersi a Nav2
    if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server non disponibile dopo 10 secondi! Uscita.");
      return;
    }
    
    RCLCPP_INFO(this->get_logger(), "Action Server trovato! Invio pose in corso...");

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SendGoalOptions();

    send_goal_options.goal_response_callback =
      std::bind(&PoseClient::goal_response_callback, this, std::placeholders::_1);

    send_goal_options.feedback_callback =
      std::bind(&PoseClient::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);

    send_goal_options.result_callback =
      std::bind(&PoseClient::result_callback, this, std::placeholders::_1);

    action_client_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  void goal_response_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr & goal_handle)
  {
    if (!goal_handle) {
      RCLCPP_ERROR(get_logger(), "Goal was rejected by the server.");
      return; 
    }
    RCLCPP_INFO(get_logger(), "Goal accepted by the server. Waiting for result...");
  }

  void feedback_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::SharedPtr goal_handle,
    const std::shared_ptr<const nav2_msgs::action::NavigateThroughPoses::Feedback> feedback)
  {
    (void)goal_handle;
    RCLCPP_INFO(
      get_logger(),
      "Feedback: distanza rimanente: %f m || numero di pose rimanenti: %d",
      feedback->distance_remaining,
      feedback->number_of_poses_remaining);
  }

  void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateThroughPoses>::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
        RCLCPP_INFO(this->get_logger(), "Operation completed successfully! Il robot e' arrivato.");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Operation failed or canceled.");
    }
  }

  rclcpp_action::Client<nav2_msgs::action::NavigateThroughPoses>::SharedPtr action_client_;
};


//funzione per leggere il file csv e mettere tutto in una matrice
std::vector<std::vector<double>> read_csv(std::ifstream & file)
{
    std::vector<std::vector<double>> values;
    std::vector<double> temp;
    std::string line = "";
    int location = 0;
    std::string remaining;
    std::string first;
    std::string second;
    std::string third;
    double x;
    double y;
    double theta;

    std::getline(file, line); //legge la prima linea di intestazione
    std::getline(file, line);//siamo alla seconda, da qua iniziano i numeri    
    
    while(line.find(',') != std::string::npos)
    {
        temp.clear();
        location = line.find(',');
        first = line.substr(0, location);
        x = std::stod(first);
        temp.push_back(x);

        remaining = line.substr(location + 1, line.length());
        location = remaining.find(',');
        second = remaining.substr(0, location);
        y = std::stod(second);
        temp.push_back(y);

        remaining = remaining.substr(location + 1, remaining.length());
        location = remaining.find(',');
        third = remaining.substr(0, location);
        theta = std::stod(third);
        temp.push_back(theta);
        
        values.push_back(temp);

        std::getline(file, line);
    }
    return values;
}



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  std::string pkg_dir = ament_index_cpp::get_package_share_directory("second_project");
  std::string file_path = pkg_dir + "/csv/goals.csv";
  std::ifstream file;
  
  file.open(file_path);
  if (!file.is_open()) {
    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "ERRORE CRITICO: Impossibile aprire il file CSV al percorso: %s", file_path.c_str());
    rclcpp::shutdown();
    return 1;
  }

  std::vector<std::vector<double>> values = read_csv(file);
  
  auto node = std::make_shared<PoseClient>();
  node->send_goal(values);
  rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}