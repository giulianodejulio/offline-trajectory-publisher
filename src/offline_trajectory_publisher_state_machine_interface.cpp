#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <Eigen/Dense>

#include <iostream>                                     //to read data from files
#include <fstream>                                      

#include "rclcpp/rclcpp.hpp"                            //to use ROS messages
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "hidro_ros2_utils/state_machine_interface.hpp" //to inherit from StateMachineInterface class

// #include "/home/hidro/solo12_ws/src/odri_ros2/odri_ros2_hardware/include/odri_ros2_hardware/robot_interface.hpp" //to use RobotCommand msg definition (?)
// #include "odri_ros2_interfaces/msg/motor_command.hpp" //to use Motorommand msg definition
#include "odri_ros2_interfaces/msg/robot_command.hpp" //to use RobotCommand msg definition


using namespace std::chrono_literals;

int rows = 0;     //number of rows of a matrix stored in a file. Used in the overloaded >> operator.
int cols = 0;     //number of cols of a matrix stored in a file. Used in the overloaded >> operator.

// Custom overload for the >> operator that reads data from a file and stores it in an Eigen::MatrixXd object
std::istream& operator>>(std::istream& is, Eigen::MatrixXd& mat)
{
  mat.resize(rows, cols);
  for (int i = 0; i < rows; ++i) {
      for (int j = 0; j < cols; j++) {
          is >> mat(i,j);
      }
  }

  return is;
}

void computeNumberOfRowsAndCols(const std::string& filename){
      // Open the file
    std::ifstream file(filename);

    // Read the matrix as a vector of vectors
    std::vector<std::vector<double>> matrix;
    std::string line;
    while (std::getline(file, line))
    {
        std::vector<double> row;
        std::stringstream ss(line);
        double value;
        while (ss >> value)
        {
            row.push_back(value);
        }
        matrix.push_back(row);
    }

    // Count the number of rows and columns
    rows = matrix.size();
    cols = matrix[0].size();

}

void openFile(std::string file_name, Eigen::MatrixXd& mat){
  std::ifstream is_file(file_name);
  if(is_file.is_open()) {
      computeNumberOfRowsAndCols(file_name);
      if((is_file >> mat)) {
          is_file.close();
          std::cout << "Loaded matrix:\n" << std::endl;
      }
      else std::cout << "Error: could not read matrix from file." << std::endl;
  }
  else std::cout << "Error: could not open file." << std::endl;
}

// namespace hidro_ros2_utils {
class OfflineTrajectoryPublisher : public hidro_ros2_utils::StateMachineInterface
{
public:
  OfflineTrajectoryPublisher() : Node("offline_trajectory_publisher_state_machine_interface"), 
                                 hidro_ros2_utils::StateMachineInterface("offline_trajectory_publisher_state_machine_interface")

  {
    initializeParameters();
    stateMachineModifications();
    openFile(params_.xs_dot_txt, xs_);
    openFile(params_.us_dot_txt, us_);
    initializePublishers();
  }

private:
    void initializeParameters(){
      declare_parameter<int>("publish_period");
      declare_parameter<std::string>("xs_dot_txt");
      declare_parameter<std::string>("us_dot_txt");
      declare_parameter<double>("kp");
      declare_parameter<double>("kd");
      declare_parameter<double>("i_sat");

      params_.publish_period = std::chrono::duration<int, std::milli>(get_parameter("publish_period").as_int());
      get_parameter("xs_dot_txt", params_.xs_dot_txt);
      get_parameter("us_dot_txt", params_.us_dot_txt);
      get_parameter("kp", params_.kp);
      get_parameter("kd", params_.kd);
      get_parameter("i_sat", params_.i_sat);
    }

    void stateMachineModifications(){
      state_machine_->addState("safe");
      state_machine_->addTransition("stop1", "running", "safe");
      state_machine_->addTransition("disable1", "safe", "idle");
      state_machine_->addTransition("enable1", "safe", "enabled");
      state_machine_->assignTransitionCallback("stop1",
                                        &OfflineTrajectoryPublisher::transStop1Callback, this);
      state_machine_->assignTransitionCallback("disable1",
                                        &OfflineTrajectoryPublisher::transDisableCallback, this);
      state_machine_->assignTransitionCallback("enable1",
                                        &OfflineTrajectoryPublisher::transEnableCallback, this); //TODO: implement it
    }

    void initializePublishers()
    {
      cmd_.motor_commands.resize(12);
      for(int i = 0; i < 12; i++) {
        cmd_.motor_commands[i].position_ref = 0.0;
        cmd_.motor_commands[i].velocity_ref = 0.0;
        cmd_.motor_commands[i].torque_ref   = 0.0;
        cmd_.motor_commands[i].kp           = 0.0;
        cmd_.motor_commands[i].kd           = 0.0;
        cmd_.motor_commands[i].i_sat        = 0.0;
      }

      publisher_ = this->create_publisher<odri_ros2_interfaces::msg::RobotCommand>("odri_cmd", 10);
      timer_ = this->create_wall_timer(
          params_.publish_period, std::bind(&OfflineTrajectoryPublisher::publishRowsCb, this));
    }

  //Offline Trajectory Publisher
    void publishRowsCb()
    {
      computeCommand();
      publisher_->publish(cmd_);
    }

    void computeCommand()
    {
      if(state_machine_->getStateActive() == "enabled") //publish the first point of the trajectory
      {
        for(int j = 0; j < 12; j++)
          {
            cmd_.motor_commands[j].position_ref = xs_(0,j+7);
            cmd_.motor_commands[j].velocity_ref = xs_(0,j+25);
            cmd_.motor_commands[j].torque_ref   = us_(0,j);
            cmd_.motor_commands[j].kp           = params_.kp;
            cmd_.motor_commands[j].kd           = params_.kd;
            cmd_.motor_commands[j].i_sat        = params_.i_sat;
          }
          // std::cout << std::endl;


      }
      //active state is "enabled"

      else if(state_machine_->getStateActive() == "running") //publish the whole trajectory
      {
        if(row_index_ < us_.rows())
        {
          for(int j = 0; j < 12; j++)
          { if(j!=2){
            cmd_.motor_commands[j].position_ref = xs_(row_index_,j+7);
            cmd_.motor_commands[j].velocity_ref = xs_(row_index_,j+25);
            cmd_.motor_commands[j].torque_ref   = us_(row_index_,j);
            cmd_.motor_commands[j].kp           = params_.kp;
            cmd_.motor_commands[j].kd           = params_.kd;
            cmd_.motor_commands[j].i_sat        = params_.i_sat;}
          }
          row_index_++;
        }
        else //publish last value in xs_ and us_
        {
          for(int j = 0; j < 12; j++)
          {
            cmd_.motor_commands[j].position_ref = xs_(row_index_,j+7);
            cmd_.motor_commands[j].velocity_ref = xs_(row_index_,j+25);
            cmd_.motor_commands[j].torque_ref   = us_(row_index_-1,j);
            cmd_.motor_commands[j].kp           = params_.kp;
            cmd_.motor_commands[j].kd           = params_.kd;
            cmd_.motor_commands[j].i_sat        = params_.i_sat;
          }
        }
      }
      //active state is "running"

      else if(state_machine_->getStateActive() == "safe") //publish only kd for a damped landing
      {
        for(int j = 0; j < 12; j++)
          {
            cmd_.motor_commands[j].position_ref = 0;
            cmd_.motor_commands[j].velocity_ref = 0;
            cmd_.motor_commands[j].torque_ref   = 0;
            cmd_.motor_commands[j].kp           = 0;
            cmd_.motor_commands[j].kd           = params_.kd;
            cmd_.motor_commands[j].i_sat        = params_.i_sat;
          }
      }
      //active state is "safe"
    }

    rclcpp::Publisher<odri_ros2_interfaces::msg::RobotCommand>::SharedPtr publisher_;
    odri_ros2_interfaces::msg::RobotCommand cmd_;
    int row_index_ = 0;
    rclcpp::TimerBase::SharedPtr timer_;
    Eigen::MatrixXd xs_;                //State trajectory matrix to be filled with the file stored in parameter xs_dot_txt
    Eigen::MatrixXd us_;                //Control trajectory matrix to be filled with the file stored in parameter us_dot_txt

    struct Params {
      std::chrono::duration<int, std::milli> publish_period;  //inverse of the rate of the ros2 node publisher named publisher_
      std::string xs_dot_txt;
      std::string us_dot_txt;
      double kp;
      double kd;
      double i_sat;
    } params_ ;

  // State Machine Interface transition callbacks
    virtual bool transEnableCallback(std::string &message) override {
      std::cout << "entered in transEnableCallback. publish_period = " << params_.publish_period << std::endl;
      return true;
    }
    virtual bool transStartCallback(std::string &message) override {
      std::cout << "entered in transStartCallback" << std::endl;
      return true;
    }
    virtual bool transDisableCallback(std::string &message) override {
      std::cout << "entered in transDisableCallback" << std::endl;
      return true;
    }
    virtual bool transStopCallback(std::string &message) override {
      std::cout << "entered in transStopCallback" << std::endl;
      return true;
    }
    bool transStop1Callback(std::string &message) {
      return true;
    }

};
// } //namespace hidro_ros2_utils

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OfflineTrajectoryPublisher>());
  // rclcpp::spin(std::make_shared<hidro_ros2_utils::OfflineTrajectoryPublisher>());
  rclcpp::shutdown();
  return 0;
}


// ros2 service call /offline_trajectory_publisher_state_machine_interface/state_transition hidro_ros2_utils/srv/TransitionCommand "command: 'enable'"
// ros2 run offline_trajectory_publisher_state_machine_interface talker --ros-args --params-file ~/solo12_ws/src/cpp_pubsub/config/params.yaml